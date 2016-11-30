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
using Windows.Media;
using System.Threading;
using Windows.Graphics.Imaging;
using System.Numerics;
using Windows.Media.FaceAnalysis;
using Windows.System;
using System.Diagnostics;

namespace KinectRgbdInteraction
{
    public sealed partial class MainPage : Page
    {
        private MqttClient client = null;
        private MediaCapture mediaCapture = null;
        private FaceTracker faceTracker = null;
        private Dictionary<MediaFrameSourceKind, MediaFrameReference> frames = null;
        private SemaphoreSlim frameProcessingSemaphore = new SemaphoreSlim(1);

#if PRINT_STATUS_MESSAGE
        private Windows.UI.Xaml.DispatcherTimer statusLogTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;
#endif

        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

        public MainPage() {
            this.InitializeComponent();

            if (localSettings.Values["mqttHostAddress"] != null)
                this.IPText.Text = localSettings.Values["mqttHostAddress"].ToString();

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
            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.Connect(Guid.NewGuid().ToString());
            }

            if (this.frames == null)
                this.frames = new Dictionary<MediaFrameSourceKind, MediaFrameReference>() {
                    { MediaFrameSourceKind.Color, null },
                    { MediaFrameSourceKind.Depth, null }
                };

            if (this.faceTracker == null)
                // setup face tracker
                this.faceTracker = Task.Run(async () => { return await FaceTracker.CreateAsync(); }).Result;

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

#if PRINT_STATUS_MESSAGE
            this.appClock.Start();
#endif
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

                    // get camera intrinsics info
                    var cameraInfo = new float[] {
                        this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.FocalLength.X,
                        this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.FocalLength.Y,
                        this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.X,
                        this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.Y
                    };

                    // get color information
                    var bitmap = SoftwareBitmap.Convert(this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Premultiplied);
                    byte[] colorBytes = new byte[bitmap.PixelWidth * bitmap.PixelHeight * 4];
                    bitmap.CopyToBuffer(colorBytes.AsBuffer());

                    // map depth to color
                    // we will only create a reduced size map as original is too large : 1920 * 1080 -> 640 * 360
                    int strideX = 3;
                    int strideY = 3;
                    int resizeWidth = 640;
                    int resizeHeight = 360;

                    Point[] colorPoints = new Point[resizeWidth * resizeHeight];
                    int row = 0;
                    int idx = 0;
                    for (int i = 0; i < colorDesc.Width * colorDesc.Height; ++i) {
                        int y = i / colorDesc.Width;
                        int x = i - y * colorDesc.Width;
                        colorPoints[idx] = new Point(x, y);
                        ++idx;
                        i += strideX;
                        if (i - row * colorDesc.Width >= colorDesc.Width) {
                            row += strideY;
                            i = row * colorDesc.Width;
                        }
                    }
                    Vector3[] points = new Vector3[colorPoints.Length];
                    coordinateMapper.UnprojectPoints(colorPoints, this.frames[MediaFrameSourceKind.Color].CoordinateSystem, points);

                    // get time info
                    var time = this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.FrameReference.SystemRelativeTime;

                    // task 1 : stream point clouds
                    var task1 = Task.Run(() => {
                        byte[] bytes = new byte[points.Length * 16];
                        int j = 0;
                        for (int i = 0; i < points.Length; ++i) {
                            var values = new float[] { points[i].X, points[i].Y, points[i].Z };
                            Buffer.BlockCopy(values, 0, bytes, j, 12); j += 12;
                            Array.Copy(colorBytes, Convert.ToInt32((colorPoints[i].Y * colorDesc.Width + colorPoints[i].X) * 4) , bytes, j, 4); j += 4;
                        }
                        this.client.Publish("/kinect/stream/points", bytes);
                    });

                    // task 2 : detect face region and stream face images
                    var task2 = Task.Run(() => {
                        // detect faces
                        var videoFrame = new VideoFrame(BitmapPixelFormat.Nv12, colorDesc.Width, colorDesc.Height);
                        (SoftwareBitmap.Convert(bitmap, BitmapPixelFormat.Nv12, BitmapAlphaMode.Ignore)).CopyTo(videoFrame.SoftwareBitmap);
                        videoFrame.RelativeTime = time;
                        var faces = Task.Run(async () => { return await this.faceTracker.ProcessNextFrameAsync(videoFrame); }).Result;

                        // header information
                        uint headerSize = 16;

                        // create byte array from each face
                        uint totalSize = 0;
                        List<byte[]> faceBytesList = new List<byte[]>();
                        foreach (DetectedFace face in faces) {
                            uint size = face.FaceBox.Width * face.FaceBox.Height * 4 + headerSize;
                            byte[] faceBytes = new byte[size];
                            totalSize += size;

                            // first 4 bytes is size of face image
                            Array.Copy(BitConverter.GetBytes(face.FaceBox.Width), 0, faceBytes, 0, 2);
                            Array.Copy(BitConverter.GetBytes(face.FaceBox.Height), 0, faceBytes, 2, 2);

                            // next 12 bytes is 3d position of face
                            var centerPoint = new Point(face.FaceBox.X + Convert.ToUInt32(face.FaceBox.Width * 0.5), face.FaceBox.Y + Convert.ToUInt32(face.FaceBox.Height * 0.5));
                            var positionVector = coordinateMapper.UnprojectPoint(centerPoint, this.frames[MediaFrameSourceKind.Color].CoordinateSystem);
                            var position = new float[] { positionVector.X, positionVector.Y, positionVector.Z };
                            Buffer.BlockCopy(position, 0, faceBytes, 4, 12);

                            // copy rgb image
                            for (int y = 0; y < face.FaceBox.Height; ++y) {
                                var srcIdx = Convert.ToInt32(((y + face.FaceBox.Y) * colorDesc.Width + face.FaceBox.X) * 4);
                                var destIdx = Convert.ToInt32((y * face.FaceBox.Width) * 3 + headerSize);
                                for (int x = 0; x < face.FaceBox.Width; ++x)
                                    Array.Copy(colorBytes, srcIdx + x * 4, faceBytes, destIdx + x * 3, 3);
                            }

                            faceBytesList.Add(faceBytes);
                        }

                        // concatenate byte arrays to send (post-processed as totalSize not known in first foreach)
                        int head = 1; // first 1 byte is number of faces
                        byte[] bytes = new byte[totalSize + head];
                        Array.Copy(BitConverter.GetBytes(faceBytesList.Count), 0, bytes, 0, head);
                        foreach (byte[] faceByte in faceBytesList) {
                            Array.Copy(faceByte, 0, bytes, head, faceByte.Length);
                            head += faceByte.Length;
                        }
                        this.client.Publish("/kinect/detected/face", bytes);
                    });

                    // stream camera intrinsics
                    byte[] camIntr = new byte[16];
                    Buffer.BlockCopy(cameraInfo, 0, camIntr, 0, 16);
                    this.client.Publish("/kinect/stream/camerainfo", camIntr);

                    task1.Wait();
                    task2.Wait();

#if PRINT_STATUS_MESSAGE
                    ++this.kinectFrameCount;
#endif

                    bitmap.Dispose();
                    coordinateMapper.Dispose();
                    this.frames[MediaFrameSourceKind.Color].Dispose();
                    this.frames[MediaFrameSourceKind.Depth].Dispose();
                    this.frames[MediaFrameSourceKind.Color] = null;
                    this.frames[MediaFrameSourceKind.Depth] = null;
                }
            }
            catch (Exception ex) {
                // TODO
            }
            finally {
                frameProcessingSemaphore.Release();
            }
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            while (!frameProcessingSemaphore.Wait(0)) continue;

            try {
                if (this.mediaCapture != null) {
                    this.mediaCapture.Dispose();
                    this.mediaCapture = null;
                }

                if (this.faceTracker != null)
                    this.faceTracker = null;

                if (this.client != null) {
                    this.client.Disconnect();
                    this.client = null;
                }
            } finally {
                frameProcessingSemaphore.Release();
            }

#if PRINT_STATUS_MESSAGE
            this.appClock.Stop();
#endif
        }
    }
}
