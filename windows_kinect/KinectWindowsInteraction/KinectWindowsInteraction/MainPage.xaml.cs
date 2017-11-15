#define PRINT_STATUS_MESSAGE

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using uPLibrary.Networking.M2Mqtt;
using Windows.Foundation;
using Windows.UI.Xaml.Controls;
using Windows.System;
using Windows.UI.ViewManagement;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Diagnostics;
using Windows.Media.Ocr;
using System.Text;
using Windows.UI.Xaml;

namespace KinectWindowsInteraction
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private MqttClient client = null;
        private Dictionary<string, Action<byte[]>> requestHandlers = null;
        private Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

        List<OcrEngine> ocrEngine = null;

#if PRINT_STATUS_MESSAGE
        private Windows.UI.Xaml.DispatcherTimer statusLogTimer = new Windows.UI.Xaml.DispatcherTimer();
        private Stopwatch appClock = new Stopwatch();
#endif

        public MainPage() {
            this.InitializeComponent();
            ApplicationView.PreferredLaunchViewSize = new Size(350, 350);
            ApplicationView.PreferredLaunchWindowingMode = ApplicationViewWindowingMode.PreferredLaunchViewSize;

            if (localSettings.Values["mqttHostAddress"] != null)
                this.IPText.Text = localSettings.Values["mqttHostAddress"].ToString();

            try { // auto start client
                if (this.requestHandlers == null) {
                    this.requestHandlers = new Dictionary<string, Action<byte[]>>() {
                        { "/kinect/request/ocr", HandleRequestOCR },
                        { "/kinect/request/webagent", HandleRequestWebAgent },
                        { "/kinect/start/ocr", HandleRequestStart },
                        { "/kinect/kill/ocr", HandleRequestKill }
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
        }

        private void StartApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            localSettings.Values["mqttHostAddress"] = this.IPText.Text;
            this.Setup(this.IPText.Text);
        }

#if PRINT_STATUS_MESSAGE
        private void StatusLogTick(object sender, object e) {
            this.MemoryMonitor.Text = "MemoryUsage: " + Convert.ToString(MemoryManager.AppMemoryUsage / 1048576);
        }
#endif

        private void Setup(string ip) {
            if (this.requestHandlers == null) {
                this.requestHandlers = new Dictionary<string, Action<byte[]>>() {
                    { "/kinect/request/ocr", HandleRequestOCR },
                    { "/kinect/request/webagent", HandleRequestWebAgent }
                };
            }

            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.MqttMsgPublishReceived += this.onMqttReceive;
                this.client.Subscribe(this.requestHandlers.Keys.ToArray(), Enumerable.Repeat(MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, this.requestHandlers.Count).ToArray());
                this.client.Connect(Guid.NewGuid().ToString());
            }

            if (this.ocrEngine == null) {
                this.ocrEngine = new List<OcrEngine> { };
                var langLists = OcrEngine.AvailableRecognizerLanguages;
                foreach (var lang in langLists)
                    ocrEngine.Add(OcrEngine.TryCreateFromLanguage(lang));
            }
        }

        private void onMqttReceive(object sender, uPLibrary.Networking.M2Mqtt.Messages.MqttMsgPublishEventArgs e) {
            if (!this.requestHandlers.ContainsKey(e.Topic)) return;
            this.requestHandlers[e.Topic](e.Message);
        }

        private async void HandleRequestOCR(byte[] message) {
            // first 2 bytes is number of images
            int numImages = BitConverter.ToInt16(message, 0);

            // get images
            List<Windows.Graphics.Imaging.SoftwareBitmap> bitmaps = new List<Windows.Graphics.Imaging.SoftwareBitmap> { };
            int k = 2;
            for (int img = 0; img < numImages; ++img) {
                int width = BitConverter.ToInt16(message, k);
                int height = BitConverter.ToInt16(message, k + 2);
                k += 4;

                // bgr -> bgra
                byte[] image = new byte[width * height * 4];
                for (int j = 0; j < image.Length; ) {
                    Buffer.BlockCopy(message, k, image, j, 3); j += 3;
                    image[j++] = 0;
                    k += 3;
                }

                // bytes -> software bitmap
                var bitmap = new Windows.Graphics.Imaging.SoftwareBitmap(Windows.Graphics.Imaging.BitmapPixelFormat.Bgra8, width, height);
                bitmap.CopyFromBuffer(image.AsBuffer());
                bitmaps.Add(bitmap);
            }

            // conduct ocr
            string results = "";
            string languageDelimiter = " &&& ";
            string imageDelimiter = " ;;; ";
            foreach (var bitmap in bitmaps) {
                if (bitmap.PixelWidth > OcrEngine.MaxImageDimension || bitmap.PixelHeight > OcrEngine.MaxImageDimension) {
                    results += imageDelimiter;
                    continue;
                }

                foreach (var engine in this.ocrEngine) {
                    var ocrResult = await engine.RecognizeAsync(bitmap);
                    results += ocrResult.Text + languageDelimiter ;
                }
                results = results.Remove(results.Length - languageDelimiter.Length);
                results += imageDelimiter;
            }
            if (results.Length != 0)
                results = results.Remove(results.Length - imageDelimiter.Length);
            // e.g results = "detected message &&& message in language 2 ;;; nothing detected in next image ;;;  ;;; detected message"

            // send results
            this.client.Publish("/kinect/response/ocr", Encoding.UTF8.GetBytes(results));
        }

        private async void HandleRequestWebAgent(byte[] message) {
            string searchFor = System.Text.Encoding.UTF8.GetString(message);
            string[] words = searchFor.Split(' ');
            if (words.Length < 2)
                return;

            string searchQuery = @"http://www." + words[0] + @".com/search?q=" + words[1];
            for (var i = 2; i < words.Length; ++i)
                searchQuery += "%20" + words[i];
            var request = await Windows.System.Launcher.LaunchUriAsync(new Uri(searchQuery));
        }

        private void HandleRequestStart(byte[] message) {
            string settings = Encoding.UTF8.GetString(message);
            this.Setup(settings);
        }

        private void HandleRequestKill(byte[] message) {
            if (this.client != null) {
                this.client.Disconnect();
                this.client = null;
            }
            Application.Current.Exit();
        }

        private void CloseApp_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e) {
            if (this.client != null) {
                this.client.Disconnect();
                this.client = null;
            }
        }
    }
}
