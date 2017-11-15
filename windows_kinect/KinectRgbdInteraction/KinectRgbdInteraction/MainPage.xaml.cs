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
using Windows.System.Display;
using System.Text;
using Windows.UI.Xaml;

namespace KinectRgbdInteraction
{
    public sealed partial class MainPage : Page
    {
        private string nameSpace = "kinect";

        private MqttClient client = null;
        private MediaCapture mediaCapture = null;
        private Dictionary<MediaFrameSourceKind, MediaFrameReference> frames = null;
        private SemaphoreSlim frameProcessingSemaphore = new SemaphoreSlim(1);

        private Point[] colorPoints = null;

        private Dictionary<string, Func<byte[], bool>> requestHandlers = null;
        private bool sendImageFlag = false;
        private bool sendImageCenters = false;
        private Point[] centersInPixel = null;

        private readonly DisplayRequest displayRequest = new DisplayRequest();

#if PRINT_STATUS_MESSAGE
        private Windows.UI.Xaml.DispatcherTimer statusLogTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;
#endif

        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

        public MainPage() {
            this.InitializeComponent();
            ApplicationView.PreferredLaunchViewSize = new Size(350, 350);
            ApplicationView.PreferredLaunchWindowingMode = ApplicationViewWindowingMode.PreferredLaunchViewSize;

            if (this.localSettings.Values["mqttHostAddress"] != null)
                this.IPText.Text = this.localSettings.Values["mqttHostAddress"].ToString();

            if (this.localSettings.Values["topicNameSpace"] != null)
                this.NSText.Text = this.localSettings.Values["topicNameSpace"].ToString();
            else
                this.NSText.Text = this.nameSpace;

            try { // auto start client
                if (this.requestHandlers == null) {
                    this.requestHandlers = new Dictionary<string, Func<byte[], bool>>() {
                        { "/" + this.NSText.Text + "/request/image", HandleRequestImage },
                        { "/" + this.NSText.Text + "/request/centers", HandleRequestImageCenters },
                        { "/" + this.NSText.Text + "/start/camera", HandleRequestStart },
                        { "/" + this.NSText.Text + "/stop/camera", HandleRequestStop },
                        { "/" + this.NSText.Text + "/kill/camera", HandleRequestKill },
                    };
                }

                if (this.client == null) {
                    this.client = new MqttClient(this.IPText.Text);
                    this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                    this.client.MqttMsgPublishReceived += this.onMqttReceive;
                    this.client.Subscribe(this.requestHandlers.Keys.ToArray(), Enumerable.Repeat(MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, this.requestHandlers.Count).ToArray());
                    this.client.Connect(Guid.NewGuid().ToString());
                }
            } catch { // failed auto start client
                this.requestHandlers = null;
                this.client = null;
            }

            this.colorPoints = new Point[640 * 360];
            int row = 0;
            int at = 0;
            for (int idx = 0; idx < this.colorPoints.Length; ++idx) {
                int y = at / 1920;
                int x = at - y * 1920;
                this.colorPoints[idx] = new Point(x, y);
                at += 3;
                if (at - row * 1920 >= 1920) {
                    row += 3;
                    at = row * 1920;
                }
            }

#if PRINT_STATUS_MESSAGE
            this.statusLogTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.statusLogTimer.Tick += StatusLogTick;
            this.statusLogTimer.Start();
#endif

            displayRequest.RequestActive();
        }

        private void StartApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            this.localSettings.Values["mqttHostAddress"] = this.IPText.Text;
            this.localSettings.Values["topicNameSpace"] = this.NSText.Text;
            this.Setup(this.IPText.Text, this.NSText.Text);
        }

#if PRINT_STATUS_MESSAGE
        private void StatusLogTick(object sender, object e) {
            this.MemoryMonitor.Text = "MemoryUsage: " + Convert.ToString(MemoryManager.AppMemoryUsage / 1048576);
            if (this.appClock.IsRunning)
                this.KinectFPS.Text = "KinectFPS: " + Convert.ToString(Convert.ToInt32(this.kinectFrameCount / this.appClock.Elapsed.TotalSeconds));
        }
#endif

        private void Setup(string ip, string ns) {
            this.nameSpace = ns;

            if (this.requestHandlers == null) {
                this.requestHandlers = new Dictionary<string, Func<byte[], bool>>() {
                    { "/" + ns + "/request/image", HandleRequestImage },
                    { "/" + ns + "/request/centers", HandleRequestImageCenters }
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
                        // this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.FocalLength.X,
                        // this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.FocalLength.Y,
                        // this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.X,
                        // this.frames[MediaFrameSourceKind.Depth].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.Y
                        this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.CameraIntrinsics.FocalLength.X * 3.0f,
                        this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.CameraIntrinsics.FocalLength.Y * 3.0f,
                        this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.X / 3.0f,
                        this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.CameraIntrinsics.PrincipalPoint.Y / 3.0f
                    };

                    // get color information
                    var bitmap = SoftwareBitmap.Convert(this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Ignore);
                    byte[] colorBytes = new byte[bitmap.PixelWidth * bitmap.PixelHeight * 4];
                    bitmap.CopyToBuffer(colorBytes.AsBuffer());

                    // get time info
                    var time = this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.FrameReference.SystemRelativeTime;

                    // map depth to color
                    // we will only create a reduced size map as original is too large : 1920 * 1080 -> 640 * 360
                    Vector3[] points = new Vector3[this.colorPoints.Length];
                    coordinateMapper.UnprojectPoints(this.colorPoints, this.frames[MediaFrameSourceKind.Color].CoordinateSystem, points);

                    // stream point clouds
                    byte[] streamBytes = new byte[points.Length * 16];
                    Parallel.ForEach(System.Collections.Concurrent.Partitioner.Create(0, points.Length),
                        (range) => {
                            int j = range.Item1 * 16;
                            for (int i = range.Item1; i < range.Item2; ++i) {
                                var values = new float[] { points[i].X, points[i].Y, points[i].Z };
                                Buffer.BlockCopy(values, 0, streamBytes, j, 12); j += 12;
                                Buffer.BlockCopy(colorBytes, Convert.ToInt32((this.colorPoints[i].Y * colorDesc.Width + this.colorPoints[i].X) * 4), streamBytes, j, 4); j += 4;
                            }
                        });
                    this.client.Publish("/" + this.nameSpace + "/stream/points", streamBytes);

                    // stream camera intrinsics
                    byte[] camIntr = new byte[16];
                    Buffer.BlockCopy(cameraInfo, 0, camIntr, 0, 16);
                    this.client.Publish("/" + this.nameSpace + "/stream/camerainfo", camIntr);

                    // other requested queues (only one is processed at each frame)

                    if (this.sendImageFlag) { // send full image if requested (full image should usually not be requested)
                        byte[] bgrColorBytes = new byte[colorDesc.Width * colorDesc.Height * 3];
                        Parallel.ForEach(System.Collections.Concurrent.Partitioner.Create(0, colorDesc.Height),
                        (range) => {
                            for (int i = range.Item1; i < range.Item2; ++i) {
                                int srcIdx = i * colorDesc.Width * 4;
                                int destIdx = i * colorDesc.Width * 3;
                                for (int x = 0; x < colorDesc.Width; ++x)
                                    Buffer.BlockCopy(colorBytes, srcIdx + x * 4, bgrColorBytes, destIdx + x * 3, 3);
                            }
                        });
                        this.client.Publish("/kinect/stream/image", bgrColorBytes);
                        this.sendImageFlag = false;
                    } else if (this.sendImageCenters) { // send image centers if requested
                        // get image centers from center pixels
                        Vector3[] positionVectors = new Vector3[this.centersInPixel.Length];
                        coordinateMapper.UnprojectPoints(this.centersInPixel, this.frames[MediaFrameSourceKind.Color].CoordinateSystem, positionVectors);

                        // Vector3 -> float[]
                        int numResults = positionVectors.Length;
                        float[] positions = new float[positionVectors.Length * 3];
                        for (int i = 0; i < positionVectors.Length; ++i)
                            positionVectors[i].CopyTo(positions, i * 3);

                        // float[] -> byte[]
                        byte[] positionBytes = new byte[positions.Length * 4 + 2];
                        Buffer.BlockCopy(BitConverter.GetBytes(numResults), 0, positionBytes, 0, 2);
                        Buffer.BlockCopy(positions, 0, positionBytes, 2, positions.Length * 4);
                        this.client.Publish("/kinect/stream/centers", positionBytes);
                        this.sendImageCenters = false;
                    }

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

        private bool HandleRequestImage(byte[] message) {
            this.sendImageFlag = true;
            return true;
        }

        private bool HandleRequestImageCenters(byte[] message) {
            if (message.Length == 0) return false;

            int numRequests = BitConverter.ToInt16(message, 0); // first 2 bytes is number of images
            if (this.centersInPixel != null)
                Array.Resize(ref this.centersInPixel, numRequests);
            else
                this.centersInPixel = new Point[numRequests];

            // parse request (8 bytes per image)
            int at = 2;
            for (int i = 0; i < numRequests; ++i) {
                int x = BitConverter.ToInt16(message, at);
                int y = BitConverter.ToInt16(message, at + 2);
                int width = BitConverter.ToInt16(message, at + 4);
                int height = BitConverter.ToInt16(message, at + 6);
                this.centersInPixel[i] = new Point(x + Convert.ToInt32(width * 0.5), y + Convert.ToInt32(height * 0.5));
                at += 8;
            }
            this.sendImageCenters = true;
            return true;
        }

        private bool HandleRequestStart(byte[] message) {
            string settingsString = Encoding.UTF8.GetString(message);
            string[] settings = settingsString.Split(';');
            this.Setup(settings[0], settings[1]);
            return true;
        }

        private bool HandleRequestStop(byte[] message) {
            this.Stop();
            Application.Current.Exit();
            return true;
        }

        private bool HandleRequestKill(byte[] message) {
            Application.Current.Exit();
            return true;
        }

        private void Stop() {
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

#if PRINT_STATUS_MESSAGE
            this.appClock.Stop();
#endif
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            Stop();
        }
    }
}
