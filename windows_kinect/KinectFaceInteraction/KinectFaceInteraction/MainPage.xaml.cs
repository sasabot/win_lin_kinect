#define PRINT_STATUS_MESSAGE

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using uPLibrary.Networking.M2Mqtt;
using Windows.Foundation;
using Windows.UI.Xaml.Controls;
using System.Threading.Tasks;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.Devices.Core;
using System.Threading;
using Windows.Graphics.Imaging;
using System.Numerics;
using Windows.System;
using System.Diagnostics;
using Windows.UI.ViewManagement;
using uPLibrary.Networking.M2Mqtt.Messages;
using Windows.Storage;

namespace KinectFaceInteraction
{
    public class FaceLog {
        public int id;
        public Vector3 facePosition;
        public bool isTracked;
        public bool foundInThisFrame;
        public int lostTrackCount;

        public FaceLog(int _id) {
            id = _id;
            facePosition = new Vector3(0, 0, 0);
            isTracked = false;
            foundInThisFrame = false;
            lostTrackCount = 0;
        }

        public void SetFoundFalse() {
            foundInThisFrame = false;
        }

        public void Update(Vector3 _facePosition) {
            facePosition = _facePosition;
            isTracked = true;
            foundInThisFrame = true;
            lostTrackCount = 0;
        }

        public void Free(int renewId) {
            id = renewId;
            facePosition.X = 0; facePosition.Y = 0; facePosition.Z = 0;
            isTracked = false;
            foundInThisFrame = false;
            lostTrackCount = 0;
        }
    }

    public class DepthMapFunction {
        public float a;
        public float b;
        public float xMin;
        public float xMax;

        public DepthMapFunction(float _a, float _b, float _xMin, float _xMax) {
            a = _a;
            b = _b;
            xMin = _xMin;
            xMax = _xMax;
        }

        public float F(float x) {
            return a * x + b;
        }
    }

    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private MqttClient client = null;
        private MediaCapture mediaCapture = null;
        private Dictionary<MediaFrameSourceKind, MediaFrameReference> frames = null;
        private SemaphoreSlim frameProcessingSemaphore = new SemaphoreSlim(1);

        private HaarRuntimeComponent.Class1 cascadeClassifier = null;
        private List<DepthMapFunction> depthMap = new List<DepthMapFunction> { };
        private Point[] mapPoints = null;

        // captures up to three faces
        private List<FaceLog> faceLog = new List<FaceLog> { new FaceLog(0), new FaceLog(1), new FaceLog(2) };
        private int nextReservedFaceId = 3;
        private Stack<int> freeFaceQueue = new Stack<int>();

        private Dictionary<string, Func<byte[], bool>> requestHandlers = null;

        // for print status (note: fps is used in track, essential)
        private Windows.UI.Xaml.DispatcherTimer statusLogTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;

        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

        public MainPage() {
            this.InitializeComponent();

            ApplicationView.PreferredLaunchViewSize = new Size(350, 350);
            ApplicationView.PreferredLaunchWindowingMode = ApplicationViewWindowingMode.PreferredLaunchViewSize;

            if (localSettings.Values["mqttHostAddress"] != null)
                this.IPText.Text = localSettings.Values["mqttHostAddress"].ToString();

            int row = 0;
            int at = 0;
            this.mapPoints = new Point[240 * 135];
            for (int i = 0; i < this.mapPoints.Length; ++i) {
                int x = at + 4;
                int y = row + 4;
                this.mapPoints[i] = new Point(x, y);
                at += 8;
                if (at > 1912) {
                    row += 8;
                    at = 0;
                }
            }

#if PRINT_STATUS_MESSAGE
            this.statusLogTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.statusLogTimer.Tick += StatusLogTick;
            this.statusLogTimer.Start();
#endif
        }

        private void StartApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            localSettings.Values["mqttHostAddress"] = this.IPText.Text;
            this.Setup(this.IPText.Text);
        }

#if PRINT_STATUS_MESSAGE
        private void StatusLogTick(object sender, object e) {
            this.MemoryMonitor.Text = "MemoryUsage: " + Convert.ToString(MemoryManager.AppMemoryUsage / 1048576);
            if (this.appClock.IsRunning)
                this.KinectFPS.Text = "KinectFPS: " + Convert.ToString(Convert.ToInt32(kinectFrameCount / this.appClock.Elapsed.TotalSeconds));
        }
#endif

        private void Setup(string ip) {
            string xml = "";
            if (this.cascadeClassifier == null) {
                Task.Run(async () => {
                    StorageFolder picturesLibrary = await KnownFolders.GetFolderForUserAsync(null /* current user */, KnownFolderId.PicturesLibrary);
                    var sampleFile = (StorageFile)await picturesLibrary.TryGetItemAsync(@"cascades\haarcascade_mcs_upperbody.xml");
                    if (sampleFile != null) {
                        using (var inputStream = await sampleFile.OpenReadAsync()) {
                            Byte[] bytes = new Byte[inputStream.Size];
                            var buffer = await inputStream.ReadAsync(bytes.AsBuffer(), (uint)inputStream.Size, Windows.Storage.Streams.InputStreamOptions.None);
                            bytes = buffer.ToArray();
                            xml = System.Text.Encoding.UTF8.GetString(bytes, 0, bytes.Length);
                        }
                    }
                }).Wait();
                this.cascadeClassifier = new HaarRuntimeComponent.Class1(xml);
            }

            if (this.requestHandlers == null) {
                this.requestHandlers = new Dictionary<string, Func<byte[], bool>>() {
                    { "/kinect/request/facetrack/bounds", HandleRequestFaceTrackBounds }
                };
            }

            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.MqttMsgPublishReceived += this.onMqttReceive;
                this.client.Subscribe(this.requestHandlers.Keys.ToArray(), Enumerable.Repeat(MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, this.requestHandlers.Count).ToArray());
                this.client.Connect(Guid.NewGuid().ToString());
            }

            if (this.frames == null)
                this.frames = new Dictionary<MediaFrameSourceKind, MediaFrameReference>() {
                    { MediaFrameSourceKind.Color, null },
                    { MediaFrameSourceKind.Depth, null }
                };

            if (this.mediaCapture == null) {
                // select device with both color and depth streams
                var cameras = Task.Run(async () => { return await MediaFrameSourceGroup.FindAllAsync(); });
                var eligible = cameras.Result.Select(c => new {
                    Group = c,
                    SourceInfos = new MediaFrameSourceInfo[] {
                    c.SourceInfos.FirstOrDefault(info => info.SourceKind == MediaFrameSourceKind.Color),
                    c.SourceInfos.FirstOrDefault(info => info.SourceKind == MediaFrameSourceKind.Depth)
                    }
                }).Where(c => c.SourceInfos[0] != null && c.SourceInfos[1] != null).ToList();
                if (eligible.Count == 0) return;
                var selected = eligible[0];

                // open device
                this.mediaCapture = new MediaCapture();
                Task.Run(async () => {
                    await this.mediaCapture.InitializeAsync(new MediaCaptureInitializationSettings {
                        SourceGroup = selected.Group,
                        SharingMode = MediaCaptureSharingMode.SharedReadOnly,
                        StreamingCaptureMode = StreamingCaptureMode.Video,
                        MemoryPreference = MediaCaptureMemoryPreference.Cpu
                    });
                }).Wait();

                // set stream callbacks
                for (int i = 0; i < selected.SourceInfos.Length; ++i) {
                    MediaFrameSourceInfo info = selected.SourceInfos[i];
                    MediaFrameSource frameSource = null;
                    if (this.mediaCapture.FrameSources.TryGetValue(info.Id, out frameSource)) {
                        var frameReader = Task.Run(async () => { return await this.mediaCapture.CreateFrameReaderAsync(frameSource); });
                        frameReader.Result.FrameArrived += FrameReader_FrameArrived;
                        var status = Task.Run(async () => { return await frameReader.Result.StartAsync(); });
                        if (status.Result != MediaFrameReaderStartStatus.Success) return;
                    }
                }
            }

            // start clock
            this.appClock.Start();
        }

        private void FrameReader_FrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args) {
            if (!frameProcessingSemaphore.Wait(0)) return;

            try {
                var frame = sender.TryAcquireLatestFrame();
                if (frame != null) this.frames[frame.SourceKind] = frame;

                if (this.frames[MediaFrameSourceKind.Color] != null && this.frames[MediaFrameSourceKind.Depth] != null) {
                    var colorDesc = this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap.LockBuffer(BitmapBufferAccessMode.Read).GetPlaneDescription(0);
                    var depthDesc = this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.SoftwareBitmap.LockBuffer(BitmapBufferAccessMode.Read).GetPlaneDescription(0);

                    // get points in 3d space
                    DepthCorrelatedCoordinateMapper coordinateMapper = this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.DepthMediaFrame.TryCreateCoordinateMapper(
                        this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.CameraIntrinsics, this.frames[MediaFrameSourceKind.Color].CoordinateSystem);

                    // get color information
                    var bitmap = SoftwareBitmap.Convert(this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Ignore);
                    byte[] colorBytes = new byte[bitmap.PixelWidth * bitmap.PixelHeight * 4];
                    bitmap.CopyToBuffer(colorBytes.AsBuffer());

                    Vector3[] depthPoints = new Vector3[this.mapPoints.Length];
                    coordinateMapper.UnprojectPoints(this.mapPoints, this.frames[MediaFrameSourceKind.Color].CoordinateSystem, depthPoints);

                    // resize image 1920x1080 -> 480x270 and apply depth filter
                    byte[] resizedGrayColorBytes = new byte[240 * 135];
                    int row = 0;
                    int at = 0;
                    for (int idx = 0; idx < resizedGrayColorBytes.Length; ++idx) {
                        var depth = depthPoints[idx];

                        // get depth bound for pixel idx
                        float depthBound = 10.0f;
                        if (float.IsNaN(depth.X))
                            depthBound = 0.0f;
                        else
                            for (int i = 0; i < this.depthMap.Count; ++i)
                                if (this.depthMap[i].xMin < depth.X && depth.X < this.depthMap[i].xMax) {
                                    depthBound = this.depthMap[i].F(depth.X);
                                    break;
                                }

                        // get color of pixel idx
                        // topLeft : at; topRight : at + strideX - 4;
                        // bottomLeft : at + rgbWidth * (strideY - 1); bottomRight : at + rgbWidth * (strideY - 1) + strideX - 4;
                        float bgr = 255;
                        if (depth.Z < depthBound)
                            bgr =
                                (Convert.ToInt16(colorBytes[at] + colorBytes[at + 28] + colorBytes[at + 53760] + colorBytes[at + 53788]
                                + colorBytes[at + 1] + colorBytes[at + 29] + colorBytes[at + 53761] + colorBytes[at + 53789]
                                + colorBytes[at + 2] + colorBytes[at + 30] + colorBytes[at + 53762] + colorBytes[at + 53790])) * 0.0833333f;
                        resizedGrayColorBytes[idx] = BitConverter.GetBytes(Convert.ToInt32(bgr))[0];

                        // iterate
                        at += 32;
                        if (at - row * 7680 > 7648) { // at - row * rgbWidth > rgbWidth - strideX
                            row += 8;
                            at = row * 7680;
                        }
                    }
                    var faces = this.cascadeClassifier.DetectMultiScale(resizedGrayColorBytes, 240, 135);

                    // debug image (optional)
                    //byte[] dbg = new byte[240 * 135 + 4];
                    //Buffer.BlockCopy(BitConverter.GetBytes(240), 0, dbg, 0, 2);
                    //Buffer.BlockCopy(BitConverter.GetBytes(135), 0, dbg, 2, 2);
                    //Buffer.BlockCopy(resizedGrayColorBytes, 0, dbg, 4, 32400);
                    //this.client.Publish("/kinect/face/debug", dbg);

                    // reset face found status
                    foreach (var log in this.faceLog)
                        log.SetFoundFalse();

                    // parse data from cascadeClassifier
                    List<Vector3> facePositionList = new List<Vector3>();
                    List<Tuple<uint, uint, uint, uint>> faceBoundsList = new List<Tuple<uint, uint, uint, uint>>();
                    at = 1;
                    int numDetectFaces = faces[0];
                    for (int j = 0; j < numDetectFaces; ++j) {
                        // parse result from C++
                        uint xj = Convert.ToUInt32(faces[at++]) * 8;
                        uint yj = Convert.ToUInt32(faces[at++]) * 8;                        
                        uint width = Convert.ToUInt32(faces[at++]) * 8;
                        // result is head + shoulder -> multiply 0.6 to get head only region
                        uint height = Convert.ToUInt32(Convert.ToInt32(faces[at++]) * 8 * 0.6);
                        // center crop image
                        xj += Convert.ToUInt32(width * 0.2);
                        width = Convert.ToUInt32(width * 0.6);

                        // get face 3d position
                        var centerPoint = new Point(xj + Convert.ToUInt32(width * 0.5), yj + Convert.ToUInt32(height * 0.5));
                        var positionVector = coordinateMapper.UnprojectPoint(centerPoint, this.frames[MediaFrameSourceKind.Color].CoordinateSystem);

                        facePositionList.Add(positionVector);
                        faceBoundsList.Add(new Tuple<uint, uint, uint, uint>(xj, yj, width, height));
                    }

                    // find face in current frame that matches log (only one face is linked to each log)
                    int[] faceCorrespondingLog = Enumerable.Repeat(-1, facePositionList.Count).ToArray();
                    float[] faceCorrespondenceScore = Enumerable.Repeat(float.MaxValue, facePositionList.Count).ToArray();
                    for (int i = 0; i < this.faceLog.Count; ++i) {
                        if (!faceLog[i].isTracked) continue; // log is currently not used

                        int likelyEnum = -1;
                        float likeliness = float.MaxValue;
                        for (int j = 0; j < facePositionList.Count; ++j) {
                            var dist = Vector3.Distance(facePositionList[j], this.faceLog[i].facePosition);
                            if (dist < 0.3 && dist < likeliness) { // it is unlikely for a face to jump 30cm between frames
                                likelyEnum = j;
                                likeliness = dist;
                            }
                        }

                        if (likelyEnum >= 0 && likeliness < faceCorrespondenceScore[likelyEnum]) {
                            if (faceCorrespondingLog[likelyEnum] >= 0)
                                this.faceLog[faceCorrespondingLog[likelyEnum]].foundInThisFrame = false; // last log was not right match, undo match
                            faceCorrespondingLog[likelyEnum] = i;
                            this.faceLog[i].foundInThisFrame = true; // this log is now matched with face
                        }
                    }

                    // check faceCorrespondingLog and create byte array from each face
                    uint totalSize = 0;
                    List<byte[]> faceBytesList = new List<byte[]>();
                    for (int j = 0; j < faceCorrespondingLog.Length; ++j) {
                        int likelyEnum = -1;
                        Vector3 positionVector = facePositionList[j];
                        if (faceCorrespondingLog[j] < 0) { // corresponding log was not yet found
                            // find likely face log from logs
                            float likeliness = float.MaxValue;
                            for (int i = 0; i < this.faceLog.Count; ++i)
                                if (this.faceLog[i].isTracked) {
                                    var dist = Vector3.Distance(positionVector, this.faceLog[i].facePosition);
                                    if (dist < 0.3 && dist < likeliness) { // it is unlikely for a face to jump 30cm between frames
                                        likelyEnum = i;
                                        likeliness = dist;
                                    }
                                }
                            if (likelyEnum >= 0 && this.faceLog[likelyEnum].foundInThisFrame)
                                continue; // detected face was somehow a duplicate of an existing region, ignore
                            if (likelyEnum < 0) // if no likely face was found
                                for (int i = 0; i < this.faceLog.Count; ++i)
                                    if (!this.faceLog[i].isTracked) { // a new track is registered (will switch to isTracked when called Update)
                                        likelyEnum = i;
                                        break;
                                    }
                            if (likelyEnum < 0) // trackable number of faces already occupied, cannot track new face
                                continue; // id will be free once existing track is lost
                        } else { // corresponding log is already found
                            likelyEnum = faceCorrespondingLog[j];
                        }

                        this.faceLog[likelyEnum].Update(positionVector);

                        uint xj = faceBoundsList[j].Item1;
                        uint yj = faceBoundsList[j].Item2;
                        uint width = faceBoundsList[j].Item3;
                        uint height = faceBoundsList[j].Item4;

                        uint size = width * height * 3 + 20;
                        byte[] faceBytes = new byte[size];
                        totalSize += size;

                        // first 4 bytes is size of face image
                        Array.Copy(BitConverter.GetBytes(width), 0, faceBytes, 0, 2);
                        Array.Copy(BitConverter.GetBytes(height), 0, faceBytes, 2, 2);

                        // next 12 bytes is 3d position of face
                        var position = new float[] { positionVector.X, positionVector.Y, positionVector.Z };
                        Buffer.BlockCopy(position, 0, faceBytes, 4, 12);

                        // next 4 bytes is face id
                        Buffer.BlockCopy(BitConverter.GetBytes(this.faceLog[likelyEnum].id), 0, faceBytes, 16, 4);

                        // copy rgb image
                        for (int y = 0; y < height; ++y) {
                            var srcIdx = Convert.ToInt32(((y + yj) * colorDesc.Width + xj) * 4);
                            var destIdx = Convert.ToInt32((y * width) * 3 + 20);
                            for (int x = 0; x < width; ++x)
                                Buffer.BlockCopy(colorBytes, srcIdx + x * 4, faceBytes, destIdx + x * 3, 3);
                        }

                        faceBytesList.Add(faceBytes);
                    }

                    // for faces that were not found in current frame, release track state
                    foreach (var log in this.faceLog)
                        if (log.isTracked && !log.foundInThisFrame) {
                            ++log.lostTrackCount;
                            // get fps, note, clock is always running when frame is being captured
                            int fps = Convert.ToInt32(kinectFrameCount / this.appClock.Elapsed.TotalSeconds);
                            if (log.lostTrackCount > 10 * fps) { // lost for ten seconds
                                log.Free(this.nextReservedFaceId);
                                ++this.nextReservedFaceId;
                            }
                        }

                    // concatenate byte arrays to send (post-processed as totalSize not known in first foreach)
                    int head = 1; // first 1 byte is number of faces
                    byte[] bytes = new byte[totalSize + 1];
                    Array.Copy(BitConverter.GetBytes(faceBytesList.Count), 0, bytes, 0, head);
                    foreach (byte[] faceByte in faceBytesList) {
                        Array.Copy(faceByte, 0, bytes, head, faceByte.Length);
                        head += faceByte.Length;
                    }
                    this.client.Publish("/kinect/detected/face", bytes);

                    ++this.kinectFrameCount;

                    bitmap.Dispose();
                    coordinateMapper.Dispose();
                    this.frames[MediaFrameSourceKind.Color].Dispose();
                    this.frames[MediaFrameSourceKind.Depth].Dispose();
                    this.frames[MediaFrameSourceKind.Color] = null;
                    this.frames[MediaFrameSourceKind.Depth] = null;
                }
            } catch (Exception ex) {
                // TODO
            } finally {
                frameProcessingSemaphore.Release();
            }
        }

        private void onMqttReceive(object sender, uPLibrary.Networking.M2Mqtt.Messages.MqttMsgPublishEventArgs e) {
            if (!this.requestHandlers.ContainsKey(e.Topic)) return;
            this.requestHandlers[e.Topic](e.Message);
        }

        private bool HandleRequestFaceTrackBounds(byte[] message) {
            while (!frameProcessingSemaphore.Wait(0)) { };

            this.depthMap.Clear();

            // make sure message is valid
            if (message.Length < 2) return false;
            int numPoints = BitConverter.ToInt16(message, 0);
            if (numPoints * 12 + 2 != message.Length) return false;

            // parse message
            if (numPoints == 1) {
                this.depthMap.Add(new DepthMapFunction(BitConverter.ToSingle(message, 6), BitConverter.ToSingle(message, 10), -10.0f, 10.0f));
            } else if (numPoints > 1) {
                this.depthMap.Add(new DepthMapFunction(BitConverter.ToSingle(message, 6), BitConverter.ToSingle(message, 10), -10.0f, BitConverter.ToSingle(message, 14)));
                int at = 18;
                for (int i = 1; i < numPoints - 1; ++i) {
                    this.depthMap.Add(new DepthMapFunction(BitConverter.ToSingle(message, at), BitConverter.ToSingle(message, at + 4),
                        BitConverter.ToSingle(message, at - 4), BitConverter.ToSingle(message, at + 8)));
                    at += 12;
                }
                this.depthMap.Add(new DepthMapFunction(BitConverter.ToSingle(message, at), BitConverter.ToSingle(message, at + 4), BitConverter.ToSingle(message, at - 4), 10.0f));
            }
            frameProcessingSemaphore.Release();

            return true;
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            while (!frameProcessingSemaphore.Wait(0)) continue;

            try {
                if (this.mediaCapture != null) {
                    this.mediaCapture.Dispose();
                    this.mediaCapture = null;
                }

                if (this.client != null) {
                    this.client.Disconnect();
                    this.client = null;
                }
            } finally {
                frameProcessingSemaphore.Release();
            }

            // stop clock
            this.appClock.Stop();
        }
    }
}
