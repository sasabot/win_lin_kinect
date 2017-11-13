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
            this.Setup(this.IPText.Text, this.NSText.Text, autoStart);
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            this.terminateCamera = true;
            this.terminateAudio = true;
            this.client.Publish("/" + this.nameSpace + "/stop/camera", new byte[1]);
            this.client.Publish("/kinect/kill/audio", new byte[1]);
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

            StartKinectApps("all", auto);
            //StartWindowsApps();
        }

        private async void StartKinectApps(string which, bool auto) {
            if (which == "all" || which == "audio") {
                var task3 = Task.Run(async () => {
                    var uri = new Uri("kinectmicrophoneinteraction:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });

                task3.Wait();
            }

            if (which == "all" || which == "camera") {
                var task1 = Task.Run(async () => {
                    var uri = new Uri("kinectrgbdinteraction:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });

                var task5 = Task.Run(async () => {
                    var uri = new Uri("kinectimagestreamer:");
                    var launched = false;
                    while (!launched) {
                        var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                        if (status == LaunchQuerySupportStatus.Available)
                            launched = await Launcher.LaunchUriAsync(uri);
                    }
                });

                task1.Wait();
                task5.Wait();
            }

            if (auto) {
                await System.Threading.Tasks.Task.Delay(1000); // wait for apps to be ready

                string msg = this.localSettings.Values["mqttHostAddress"].ToString();
                msg += ";" + this.localSettings.Values["topicNameSpace"].ToString();

                if (which == "all" || which == "camera")
                    this.client.Publish("/" + this.nameSpace + "/start/camera", Encoding.UTF8.GetBytes(msg));

                if (which == "all" || which == "audio") {
                    msg += ";" + this.localSettings.Values["languageSettings"].ToString();
                    msg += ";" + this.localSettings.Values["grammarSettings"].ToString();
                    this.client.Publish("/kinect/start/audio", Encoding.UTF8.GetBytes(msg));
                }
            }
        }

        private void StartWindowsApps() {
            var task2 = Task.Run(async () => {
                var uri = new Uri("kinectwindowsinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            var task4 = Task.Run(async () => {
                var uri = new Uri("kinectfaceinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            task2.Wait();
            task4.Wait();
        }

        private async void Check(object sender, object e) {
            if (!appTimerSemaphore.Wait(0) || !appTimerAudioSemaphore.Wait(0)) return;

            // restart apps if under freeze (restart one at a time)
            if (!this.terminateCamera && this.appClock.IsRunning && this.appClock.Elapsed.TotalMilliseconds - this.lastStreamCall > 1000) {
                this.terminateCamera = true;
                this.client.Publish("/" + this.nameSpace + "/kill/camera", new byte[1]);
                await System.Threading.Tasks.Task.Delay(1000); // wait for kill
                StartKinectApps("camera", true); // re-launch app
                this.appClock.Stop();
                this.terminateCamera = false;
            }
            else if (!this.terminateAudio && this.appClockAudio.IsRunning && this.appClockAudio.Elapsed.TotalMilliseconds - this.lastAudioCall > 1000) {
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
