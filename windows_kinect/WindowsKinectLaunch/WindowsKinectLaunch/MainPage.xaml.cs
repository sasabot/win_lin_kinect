using System;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.System;
using Windows.UI.ViewManagement;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Collections.Generic;
using Windows.System.Display;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using Windows.Media.Playback;

namespace WindowsKinectLaunch
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private string nameSpace = "kinect";
        private MqttClient client = null;
        private Dictionary<string, Func<byte[], bool>> requestHandlers = null;
        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;
        private readonly DisplayRequest displayRequest = new DisplayRequest();
        private Stopwatch appClock = new Stopwatch();
        private Stopwatch appClockAudio = new Stopwatch();
        private Windows.UI.Xaml.DispatcherTimer checkTimer = new Windows.UI.Xaml.DispatcherTimer();
        private SemaphoreSlim appTimerSemaphore = new SemaphoreSlim(1);
        private SemaphoreSlim appTimerAudioSemaphore = new SemaphoreSlim(1);

        private double lastStreamCall;
        private double lastAudioCall;
        private bool terminateCamera = false;
        private bool terminateAudio = false;

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

            if (this.localSettings.Values["languageSettings"] != null)
                this.LangText.Text = this.localSettings.Values["languageSettings"].ToString();
            else
                this.LangText.Text = "en-US";

            if (this.localSettings.Values["grammarSettings"] != null)
                this.GrammarText.Text = this.localSettings.Values["grammarSettings"].ToString();
            else
                this.GrammarText.Text = "SpeechGrammar.xml";

            if (this.localSettings.Values["otherApps"] != null)
                this.OtherText.Text = this.localSettings.Values["otherApps"].ToString();

            this.displayRequest.RequestActive();
        }

        private void StartApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            bool autoStart = true;
            if (this.IPText.Text != (string)this.localSettings.Values["mqttHostAddress"])
                autoStart = false; // must manually type IP address for start
            this.localSettings.Values["mqttHostAddress"] = this.IPText.Text;
            this.localSettings.Values["topicNameSpace"] = this.NSText.Text;
            this.localSettings.Values["languageSettings"] = this.LangText.Text;
            this.localSettings.Values["grammarSettings"] = this.GrammarText.Text;
            this.localSettings.Values["otherApps"] = this.OtherText.Text;
            this.Setup(this.IPText.Text, this.NSText.Text, autoStart);
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            this.terminateCamera = true;
            this.terminateAudio = true;
            this.client.Publish("/" + this.nameSpace + "/stop/camera", new byte[1]);
            this.client.Publish("/kinect/kill/audio", new byte[1]);
            this.client.Publish("/kinect/kill/ocr", new byte[1]);
            this.appClock.Stop();
            this.appClockAudio.Stop();
        }

        private void Setup(string ip, string ns, bool auto) {
            this.nameSpace = ns;

            if (this.requestHandlers == null) {
                this.requestHandlers = new Dictionary<string, Func<byte[], bool>>() {
                    {"/" + this.nameSpace + "/stream/camerainfo", UpdateLastStreamCall },
                    {"/kinect/audio/alive", UpdateLastAudioCall}
                };
            }

            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.MqttMsgPublishReceived += this.onMqttReceive;
                this.client.Subscribe(this.requestHandlers.Keys.ToArray(), Enumerable.Repeat(MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, this.requestHandlers.Count).ToArray());
                this.client.Connect(Guid.NewGuid().ToString());
            }

            this.checkTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.checkTimer.Tick += Check;
            this.checkTimer.Start();

            string kinectapps = "";
            if ((bool)this.CameraApp.IsChecked)
                kinectapps += "camera";
            if ((bool)this.AudioApp.IsChecked)
                kinectapps += "audio";

            string windowsapps = "";
            if ((bool)this.OcrApp.IsChecked)
                windowsapps += "ocr";
            if ((bool)this.FaceApp.IsChecked)
                windowsapps += "face";
            if ((bool)this.OtherApp.IsChecked)
                windowsapps += "other";

            StartKinectApps(kinectapps, auto);
            StartWindowsApps(windowsapps, auto);
        }

        private async void StartKinectApps(string which, bool auto) {
            if (which.Contains("audio")) {
                var task = Task.Run(async () => {
                    var uri = new Uri("kinectmicrophoneinteraction:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });
                task.Wait();
            }

            if (which.Contains("camera")) {
                var task1 = Task.Run(async () => {
                    var uri = new Uri("kinectrgbdinteraction:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });
                var task2 = Task.Run(async () => {
                    var uri = new Uri("kinectimagestreamer:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });

                task1.Wait();
                task2.Wait();
            }

            if (auto) {
                await System.Threading.Tasks.Task.Delay(1000); // wait for apps to be ready

                string msg = this.localSettings.Values["mqttHostAddress"].ToString();
                msg += ";" + this.localSettings.Values["topicNameSpace"].ToString();

                if (which.Contains("camera"))
                    this.client.Publish("/" + this.nameSpace + "/start/camera", Encoding.UTF8.GetBytes(msg));

                if (which.Contains("audio")) {
                    msg += ";" + this.localSettings.Values["languageSettings"].ToString();
                    msg += ";" + this.localSettings.Values["grammarSettings"].ToString();
                    this.client.Publish("/kinect/start/audio", Encoding.UTF8.GetBytes(msg));
                }
            }
        }

        private async void StartWindowsApps(string which, bool auto) {
            if (which.Contains("ocr")) {
                var task = Task.Run(async () => {
                    var uri = new Uri("kinectwindowsinteraction:");
                    var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                    if (status == LaunchQuerySupportStatus.Available)
                        await Launcher.LaunchUriAsync(uri);
                });
                task.Wait();
            }
            
            if (which.Contains("face")) {
                var task = Task.Run(async () => {
                    var uri = new Uri("kinectfaceinteraction:");
                    var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                    if (status == LaunchQuerySupportStatus.Available)
                        await Launcher.LaunchUriAsync(uri);
                });
                task.Wait();
            }

            if (which.Contains("other")) {
                string appName = this.OtherText.Text;
                var task = Task.Run(async () => {
                    var uri = new Uri(appName + ":");
                    var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                    if (status == LaunchQuerySupportStatus.Available)
                        await Launcher.LaunchUriAsync(uri);
                });
                task.Wait();
            }

            if (auto) {
                await System.Threading.Tasks.Task.Delay(1000); // wait for apps to be ready

                string msg = this.localSettings.Values["mqttHostAddress"].ToString();

                if (which.Contains("ocr"))
                    this.client.Publish("/kinect/start/ocr", Encoding.UTF8.GetBytes(msg));

                // face is currently not maintainanced
            }
        }

        private async void Check(object sender, object e) {
            if (!appTimerSemaphore.Wait(0) || !appTimerAudioSemaphore.Wait(0)) return;

            // restart apps if under freeze (restart one at a time)
            if (!this.terminateCamera && this.appClock.IsRunning && this.appClock.Elapsed.TotalMilliseconds - this.lastStreamCall > 3000) {
                // notify error first
                BackgroundMediaPlayer.Current.SetUriSource(new Uri("ms-winsoundevent:Notification.Looping.Alarm10"));
                BackgroundMediaPlayer.Current.Play();
                this.terminateCamera = true;
                // when an app freezes, always freezes from camera, restart client in case is full client freeze
                try {
                    this.client.Disconnect();
                    this.client = null;
                    this.client = new MqttClient((string)this.localSettings.Values["mqttHostAddress"]);
                    this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                    this.client.MqttMsgPublishReceived += this.onMqttReceive;
                    this.client.Subscribe(this.requestHandlers.Keys.ToArray(), Enumerable.Repeat(MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, this.requestHandlers.Count).ToArray());
                    this.client.Connect(Guid.NewGuid().ToString());
                    await System.Threading.Tasks.Task.Delay(1000); // wait for connection recover
                    // kill and restart camera
                    this.client.Publish("/" + this.nameSpace + "/kill/camera", new byte[1]);
                    await System.Threading.Tasks.Task.Delay(1000); // wait for kill
                    StartKinectApps("camera", true); // re-launch app
                    this.appClock.Stop();
                    this.terminateCamera = false;
                } catch {
                    // fatal restart!
                    BackgroundMediaPlayer.Current.SetUriSource(new Uri("ms-winsoundevent:Notification.Looping.Alarm8"));
                    BackgroundMediaPlayer.Current.Play();
                    await System.Threading.Tasks.Task.Delay(5000); // wait for music play to finish
                    Application.Current.Exit();
                }
            }
            else if (!this.terminateAudio && this.appClockAudio.IsRunning && this.appClockAudio.Elapsed.TotalMilliseconds - this.lastAudioCall > 5000) {
                // an audio-only freeze is unlikely a full client freeze as camera app should freeze first, no need for client restart
                BackgroundMediaPlayer.Current.SetUriSource(new Uri("ms-winsoundevent:Notification.Looping.Alarm9"));
                BackgroundMediaPlayer.Current.Play();
                this.terminateAudio = true;
                this.client.Publish("/kinect/kill/audio", new byte[1]);
                await System.Threading.Tasks.Task.Delay(1000); // wait for kill
                StartKinectApps("audio", true); // re-launch app
                this.appClockAudio.Stop();
                this.terminateAudio = false;
            }

            appTimerSemaphore.Release();
            appTimerAudioSemaphore.Release();
        }

        private void onMqttReceive(object sender, uPLibrary.Networking.M2Mqtt.Messages.MqttMsgPublishEventArgs e) {
            if (!this.requestHandlers.ContainsKey(e.Topic)) return;
            this.requestHandlers[e.Topic](e.Message);
        }

        private bool UpdateLastStreamCall(byte[] message) {
            if (!appTimerSemaphore.Wait(0)) return true;

            if (!this.terminateCamera && !this.appClock.IsRunning)
                this.appClock.Start();
            this.lastStreamCall = this.appClock.Elapsed.TotalMilliseconds;
            appTimerSemaphore.Release();
            return true;
        }

        private bool UpdateLastAudioCall(byte[] message) {
            if (!appTimerAudioSemaphore.Wait(0)) return true;

            if (!this.terminateAudio && !this.appClockAudio.IsRunning)
                this.appClockAudio.Start();
            this.lastAudioCall = this.appClockAudio.Elapsed.TotalMilliseconds;
            appTimerAudioSemaphore.Release();
            return true;
        }
    }
}
