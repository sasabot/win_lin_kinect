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
using System.Threading;
using Windows.Graphics.Imaging;
using Windows.System;
using System.Diagnostics;
using Windows.UI.ViewManagement;
using uPLibrary.Networking.M2Mqtt.Messages;
using Windows.System.Display;
using System.Text;
using Windows.UI.Xaml;
using Windows.Media.Playback;

namespace KinectImageStreamer
{
    public sealed partial class MainPage : Page
    {
        private string nameSpace = "kinect";
        private int expectedFPS = 30;

        private MqttClient client = null;
        private MediaCapture mediaCapture = null;
        private Dictionary<MediaFrameSourceKind, MediaFrameReference> frames = null;
        private SemaphoreSlim frameProcessingSemaphore = new SemaphoreSlim(1);

        private Dictionary<string, Action<byte[]>> requestHandlers = null;

        private readonly DisplayRequest displayRequest = new DisplayRequest();

#if PRINT_STATUS_MESSAGE
        private Windows.UI.Xaml.DispatcherTimer statusLogTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;
#endif

        private Windows.UI.Xaml.DispatcherTimer networkTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClockNetwork = new Stopwatch();
        private double lastNetworkCall;
        bool restartingCamera = false;

        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

        public MainPage() {
            this.InitializeComponent();
            ApplicationView.PreferredLaunchViewSize = new Size(350, 350);
            ApplicationView.PreferredLaunchWindowingMode = ApplicationViewWindowingMode.PreferredLaunchViewSize;

            if (this.localSettings.Values["mqttHostAddress"] != null)
                this.IPText.Text = localSettings.Values["mqttHostAddress"].ToString();

            if (this.localSettings.Values["topicNameSpace"] != null)
                this.NSText.Text = this.localSettings.Values["topicNameSpace"].ToString();
            else
                this.NSText.Text = this.nameSpace;

            if (this.localSettings.Values["expectedFPS"] != null)
                this.FPSText.Text = this.localSettings.Values["expectedFPS"].ToString();

            try { // auto start client
                if (this.requestHandlers == null) {
                    this.requestHandlers = new Dictionary<string, Action<byte[]>>() {
                        { "/network/alive", UpdateLastNetworkCall },
                        { "/" + this.NSText.Text + "/start/camera/rgb", HandleRequestStart },
                        { "/" + this.NSText.Text + "/stop/camera/rgb", HandleRequestStop },
                        { "/" + this.NSText.Text + "/kill/camera/rgb", HandleRequestKill },
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

#if PRINT_STATUS_MESSAGE
            this.statusLogTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.statusLogTimer.Tick += StatusLogTick;
            this.statusLogTimer.Start();
#endif
            this.networkTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.networkTimer.Tick += NetworkTick;
            this.networkTimer.Start();

            displayRequest.RequestActive();
        }

        private void StartApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            localSettings.Values["mqttHostAddress"] = this.IPText.Text;
            localSettings.Values["topicNameSpace"] = this.NSText.Text;
            this.localSettings.Values["expectedFPS"] = this.FPSText.Text;
            Int32.TryParse(this.FPSText.Text, out this.expectedFPS);
            this.Setup(this.IPText.Text, this.NSText.Text);
        }

#if PRINT_STATUS_MESSAGE
        private void StatusLogTick(object sender, object e) {
            this.MemoryMonitor.Text = "MemoryUsage: " + Convert.ToString(MemoryManager.AppMemoryUsage / 1048576);
            if (this.appClock.IsRunning)
                this.KinectFPS.Text = "KinectFPS: " + Convert.ToString(Convert.ToInt32(this.kinectFrameCount / this.appClock.Elapsed.TotalSeconds));
        }
#endif
        private void NetworkTick(object sender, object e) {
            if (!this.restartingCamera && this.appClockNetwork.IsRunning && this.appClockNetwork.Elapsed.TotalMilliseconds - this.lastNetworkCall > 5000)
                Application.Current.Exit();
        }

        private void Setup(string ip, string ns) {
            this.nameSpace = ns;

            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.Connect(Guid.NewGuid().ToString());
            }

            if (this.frames == null)
                this.frames = new Dictionary<MediaFrameSourceKind, MediaFrameReference>() {
                    { MediaFrameSourceKind.Color, null }
                };

            while (this.mediaCapture == null) {
                // select device with both color and depth streams
                var cameras = Task.Run(async () => { return await MediaFrameSourceGroup.FindAllAsync(); });
                var eligible = cameras.Result.Select(c => new {
                    Group = c,
                    SourceInfos = new MediaFrameSourceInfo[] {
                    c.SourceInfos.FirstOrDefault(info => info.SourceKind == MediaFrameSourceKind.Color),
                    c.SourceInfos.FirstOrDefault(info => info.SourceKind == MediaFrameSourceKind.Depth)
                    }
                }).Where(c => c.SourceInfos[0] != null && c.SourceInfos[1] != null).ToList();
                if (eligible.Count == 0) { // retry 1 second later
                    this.restartingCamera = true;
                    BackgroundMediaPlayer.Current.SetUriSource(new Uri("ms-winsoundevent:Notification.IM"));
                    BackgroundMediaPlayer.Current.Play();
                    Task.Run(async () => { await System.Threading.Tasks.Task.Delay(1000); }).Wait();
                    continue;
                }
                this.restartingCamera = false;
                this.lastNetworkCall = this.appClockNetwork.Elapsed.TotalMilliseconds;
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
                int nowFPS = Convert.ToInt32(this.kinectFrameCount / this.appClock.Elapsed.TotalSeconds);
                if (nowFPS > this.expectedFPS) return;

                var frame = sender.TryAcquireLatestFrame();
                if (frame != null) this.frames[frame.SourceKind] = frame;

                if (this.frames[MediaFrameSourceKind.Color] != null) {
                    var colorDesc = this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap.LockBuffer(BitmapBufferAccessMode.Read).GetPlaneDescription(0);

                    // get color information
                    var bitmap = SoftwareBitmap.Convert(this.frames[MediaFrameSourceKind.Color].VideoMediaFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Ignore);
                    byte[] colorBytes = new byte[bitmap.PixelWidth * bitmap.PixelHeight * 4];
                    bitmap.CopyToBuffer(colorBytes.AsBuffer());

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
                    this.client.Publish("/" + this.nameSpace + "/stream/image/hd", bgrColorBytes);

#if PRINT_STATUS_MESSAGE
                    ++this.kinectFrameCount;
#endif

                    bitmap.Dispose();
                    this.frames[MediaFrameSourceKind.Color].Dispose();
                    this.frames[MediaFrameSourceKind.Color] = null;
                }
            }
            catch (Exception ex) {
                // TODO
            }
            finally {
                frameProcessingSemaphore.Release();
            }
        }

        private void onMqttReceive(object sender, uPLibrary.Networking.M2Mqtt.Messages.MqttMsgPublishEventArgs e) {
            if (!this.requestHandlers.ContainsKey(e.Topic)) return;
            this.requestHandlers[e.Topic](e.Message);
        }

        private void HandleRequestStart(byte[] message) {
            string settingsString = Encoding.UTF8.GetString(message);
            string[] settings = settingsString.Split(';');
            this.Setup(settings[0], settings[1]);
        }

        private void HandleRequestStop(byte[] message) {
            this.Stop();
            Application.Current.Exit();
        }

        private void HandleRequestKill(byte[] message) {
            Application.Current.Exit();
        }

        private void UpdateLastNetworkCall(byte[] message) {
            if (!this.appClockNetwork.IsRunning)
                this.appClockNetwork.Start();
            this.lastNetworkCall = this.appClockNetwork.Elapsed.TotalMilliseconds;
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
