#define PRINT_STATUS_MESSAGE

using Microsoft.Kinect;
using System;
using System.Windows;
using System.Speech.Recognition;
using System.IO;
using uPLibrary.Networking.M2Mqtt;
using System.ComponentModel;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;
using System.Timers;
using System.Windows.Threading;

namespace KinectMicrophoneInteraction
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private MqttClient client = null;
        private KinectSensor kinectSensor = null;
        private AudioBeamFrameReader audioReader = null;
        private SpeechRecognitionEngine dictationSpeechEngine = null;
        private SpeechRecognitionEngine templateSpeechEngine = null;

#if PRINT_STATUS_MESSAGE
        private DispatcherTimer statusLogTimer = null;
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;
#endif

        public MainWindow() {
            InitializeComponent();

            this.IPText.Text = Properties.Settings.Default.Ip;
            this.LanguageText.Text = Properties.Settings.Default.Language;
            this.GrammarText.Text = Properties.Settings.Default.Grammar;

#if PRINT_STATUS_MESSAGE
            this.statusLogTimer = new DispatcherTimer();
            this.statusLogTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.statusLogTimer.Tick += StatusLogTick;
            this.statusLogTimer.Start();
#endif
        }

        private void StartApp_Click(object sender, RoutedEventArgs e) {
            Properties.Settings.Default.Ip = this.IPText.Text;
            Properties.Settings.Default.Language = this.LanguageText.Text;
            Properties.Settings.Default.Grammar = this.GrammarText.Text;
            Properties.Settings.Default.Save();
            this.Setup(this.IPText.Text, this.LanguageText.Text, this.GrammarText.Text);
        }

#if PRINT_STATUS_MESSAGE
        private void StatusLogTick(object sender, object e) {
            this.MemoryMonitor.Text = "MemoryUsage: " + Convert.ToString(System.Diagnostics.Process.GetCurrentProcess().WorkingSet64 / 1048576);
            if (this.appClock.IsRunning)
                this.KinectFPS.Text = "KinectFPS: " + Convert.ToString(Convert.ToInt32(kinectFrameCount / this.appClock.Elapsed.TotalSeconds));
        }
#endif

        private void Setup(string ip, string language, string grammar) {
            if (this.client == null) {
                this.client = new MqttClient(ip);
                this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                this.client.Connect(Guid.NewGuid().ToString());
            }

            if (this.dictationSpeechEngine == null) {
                try {
                    this.dictationSpeechEngine = new SpeechRecognitionEngine(new System.Globalization.CultureInfo(language));
                    this.dictationSpeechEngine.SpeechRecognized += this.DicatationSpeechRecognition_SpeechRecognized;
                    this.dictationSpeechEngine.LoadGrammar(new DictationGrammar());
                    this.dictationSpeechEngine.SetInputToDefaultAudioDevice();
                    this.dictationSpeechEngine.RecognizeAsync(RecognizeMode.Multiple);
                } catch { this.LanguageStatus.Text = "failed load dictation engine"; }
            }

            if (this.templateSpeechEngine == null) {
                RecognizerInfo ri = null;
                try {
                    foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
                        if (language.Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase)) {
                            ri = recognizer;
                            break;
                        }

                    this.templateSpeechEngine = new SpeechRecognitionEngine(ri.Id);
                    grammar = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + @"\Grammar\" + grammar;

                    if (File.Exists(grammar))
                        try {
                            this.templateSpeechEngine.LoadGrammar(new Grammar(grammar));
                            this.templateSpeechEngine.SpeechRecognized += this.TemplateSpeechRecognition_FrameArrived;
                            this.templateSpeechEngine.SpeechRecognitionRejected += this.TemplateSpeechRecognition_Rejected;
                            this.templateSpeechEngine.SetInputToDefaultAudioDevice();
                            this.templateSpeechEngine.RecognizeAsync(RecognizeMode.Multiple);
                        }
                        catch { this.GrammarStatus.Text = "failed load template engine"; }
                    else
                        this.GrammarStatus.Text = "failed load grammar file";

                } catch { this.GrammarStatus.Text = "recognizer is null"; }
            }

            if (this.kinectSensor == null) {
                this.kinectSensor = KinectSensor.GetDefault();

                AudioSource audioSource = this.kinectSensor.AudioSource;
                this.audioReader = audioSource.OpenReader();
                this.audioReader.FrameArrived += this.Audio_FrameArrived;
                this.kinectSensor.Open();
            }

#if PRINT_STATUS_MESSAGE
            this.appClock.Start();
#endif
        }

        private void Audio_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e) {
            AudioBeamFrameReference frameReference = e.FrameReference;

            using (var frameList = frameReference.AcquireBeamFrames()) {
                if (frameList == null) return;
                IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;
            }

            ++this.kinectFrameCount;
        }

        private void TemplateSpeechRecognition_FrameArrived(object sender, SpeechRecognizedEventArgs e) {
            const double ConfidenceThreshold = 0.3;
            if (e.Result.Confidence >= ConfidenceThreshold) {
                this.client.Publish("/kinect/detected/speech/template", Encoding.UTF8.GetBytes(e.Result.Semantics.Value.ToString()));
            }
            this.TemplateStatus.Text = e.Result.Semantics.Value.ToString() + "(" + e.Result.Confidence + ")";
        }

        private void TemplateSpeechRecognition_Rejected(object sender, SpeechRecognitionRejectedEventArgs e) {
            this.TemplateStatus.Text = "";
        }

        private void DicatationSpeechRecognition_SpeechRecognized(object sender, SpeechRecognizedEventArgs e) {
            const double ConfidenceThreshold = 0.3;
            if (e.Result.Confidence >= ConfidenceThreshold) {
                this.client.Publish("/kinect/detected/speech/dictation", Encoding.UTF8.GetBytes(e.Result.Text));
            }
            this.DictationStatus.Text = e.Result.Text + "(" + e.Result.Confidence + ")";
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e) {
            if (this.kinectSensor != null) {
                this.kinectSensor.Close();
                this.audioReader.FrameArrived -= this.Audio_FrameArrived;
                this.audioReader.Dispose();
                this.kinectSensor = null;
            }

            if (this.dictationSpeechEngine != null) {
                this.dictationSpeechEngine.SpeechRecognized -= this.DicatationSpeechRecognition_SpeechRecognized;
                this.dictationSpeechEngine.RecognizeAsyncStop();
                this.dictationSpeechEngine.Dispose();
                this.dictationSpeechEngine = null;
            }

            if (this.templateSpeechEngine != null) {
                this.templateSpeechEngine.SpeechRecognized -= this.TemplateSpeechRecognition_FrameArrived;
                this.templateSpeechEngine.SpeechRecognitionRejected -= this.TemplateSpeechRecognition_Rejected;
                this.templateSpeechEngine.RecognizeAsyncStop();
                this.templateSpeechEngine.Dispose();
                this.templateSpeechEngine = null;
            }

            if (this.client != null) {
                this.client.Disconnect();
                this.client = null;
            }
        }

    }
}
