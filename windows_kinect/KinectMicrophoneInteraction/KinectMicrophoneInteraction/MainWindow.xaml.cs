#define PRINT_STATUS_MESSAGE
//#define PUBLISH_RAW_AUDIO

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
using Microsoft.Win32;
using System.Reflection;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Speech.Synthesis;
using System.Threading;

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
        private SpeechSynthesizer speechSynthesizer = null;
        private string ttsTopic = "/kinect/request/tts";

#if PRINT_STATUS_MESSAGE
        private DispatcherTimer statusLogTimer = null;
        private Stopwatch appClock = new Stopwatch();
        private uint kinectFrameCount = 0;
#endif

        public MainWindow() {
            InitializeComponent();
            Closing += MainWindow_Closing;

            this.IPText.Text = Properties.Settings.Default.Ip;
            this.LanguageText.Text = Properties.Settings.Default.Language;
            this.GrammarText.Text = Properties.Settings.Default.Grammar;

#if PRINT_STATUS_MESSAGE
            this.statusLogTimer = new DispatcherTimer();
            this.statusLogTimer.Interval = TimeSpan.FromMilliseconds(100);
            this.statusLogTimer.Tick += StatusLogTick;
            this.statusLogTimer.Start();
#endif
            try { // auto start client
                if (this.client == null) {
                    this.client = new MqttClient(this.IPText.Text);
                    this.client.ProtocolVersion = MqttProtocolVersion.Version_3_1;
                    this.client.MqttMsgPublishReceived += this.TextToSpeech;
                    this.client.Subscribe(new string[] { ttsTopic, "/kinect/start/audio", "/kinect/kill/audio" },
                        new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE });
                    this.client.Connect(Guid.NewGuid().ToString());
                }
            } catch { // failed auto start client
                this.client = null;
            }
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
                this.client.MqttMsgPublishReceived += this.TextToSpeech;
                this.client.Subscribe(new string[] { ttsTopic }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE });
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
                    string exe = Assembly.GetExecutingAssembly().Location;
                    grammar = exe.Substring(0, exe.LastIndexOf('\\')) + @"\..\..\Grammar\" + grammar;

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

            if (this.speechSynthesizer == null) {
                this.speechSynthesizer = new SpeechSynthesizer();
                this.speechSynthesizer.SelectVoiceByHints(VoiceGender.Female, VoiceAge.Teen);
                this.speechSynthesizer.SetOutputToDefaultAudioDevice();
                this.speechSynthesizer.SpeakCompleted += this.SpeechEnd_FrameArrived;
            }

            if (this.kinectSensor == null) {
                try {
                    this.kinectSensor = KinectSensor.GetDefault();

                    AudioSource audioSource = this.kinectSensor.AudioSource;
                    this.audioReader = audioSource.OpenReader();
                    this.audioReader.FrameArrived += this.Audio_FrameArrived;
                    this.kinectSensor.Open();
                } catch { }
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

                foreach (var frame in subFrameList) {
                    if (frame.BeamAngleConfidence > 0.3) {
                        byte[] bytes = new byte[4];
                        Buffer.BlockCopy(BitConverter.GetBytes(frame.BeamAngle), 0, bytes, 0, 4);
                        this.client.Publish("/kinect/detected/audio", bytes);
                    }

#if PUBLISH_RAW_AUDIO
                    byte[] audioBuffer = new byte[this.kinectSensor.AudioSource.SubFrameLengthInBytes];
                    frame.CopyFrameDataToArray(audioBuffer);
                    byte[] convertedBuffer = new byte[audioBuffer.Length >> 1];
                    ConvertKinectAudioStream(audioBuffer, convertedBuffer);
                    this.client.Publish("/kinect/stream/rawaudio", convertedBuffer);
#endif
                    ++this.kinectFrameCount;
                }
            }
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

        private void TextToSpeech(object sender, uPLibrary.Networking.M2Mqtt.Messages.MqttMsgPublishEventArgs e) {
            if (e.Topic == "/kinect/start/audio") {
                string settingsString = Encoding.UTF8.GetString(e.Message);
                string[] settings = settingsString.Split(';');
                ThreadStart ts = delegate () {
                    Dispatcher.BeginInvoke((Action)delegate () {
                        this.LanguageText.Text = settings[1];
                        this.GrammarText.Text = settings[2];
                        Properties.Settings.Default.Language = this.LanguageText.Text;
                        Properties.Settings.Default.Grammar = this.GrammarText.Text;
                        Properties.Settings.Default.Save();
                    });
                };
                Thread t = new Thread(ts);
                t.Start();
                this.Setup(settings[0], settings[1], settings[2]);
            } else if (e.Topic == "/kinect/kill/audio") {
                ThreadStart ts = delegate () {
                    Dispatcher.BeginInvoke((Action)delegate () {
                        Close();
                        System.Windows.Application.Current.Shutdown();
                    });
                };
                Thread t = new Thread(ts);
                t.Start();
            }

            if (e.Topic != ttsTopic) return;

            this.speechSynthesizer.SpeakAsyncCancelAll();

            // example of speech
            // raw: "U--m... yeah?"
            // ssml: "<prosody rate=\"x-slow\">Um</prosody><break time=\"0.5s\"/><break time=\"0.5s\"/> yeah?"
            string rawString = Encoding.UTF8.GetString(e.Message);

            if (e.Message.Length == 0)
                return; // when command only quits speech

            // check if rawString is ssml
            if (rawString.Contains("<speak version")) {
                // remove voice tag
                if (rawString.Contains("<voice")) {
                    int voiceStart = rawString.IndexOf("<voice", 0);
                    int voiceEnd = rawString.IndexOf(">", voiceStart);
                    rawString = rawString.Remove(voiceStart, voiceEnd - voiceStart + 1);
                    voiceStart = rawString.IndexOf("</voice>");
                    if (voiceStart >= 0)
                        rawString = rawString.Remove(voiceStart, 8);
                }
                Prompt speechssml =
                    new Prompt("<?xml version=\"1.0\"?>" + rawString, SynthesisTextFormat.Ssml);
                this.speechSynthesizer.SpeakAsync(speechssml);
                return;
            }

            // else, rawString is string message

            // infer language
            string lang;
            if (e.Message.Length == rawString.Length) lang = "en-US";
            else lang = "ja-JP";

            // convert "--" syntax to x-slow
            while (true) {
                int getDashFromHead = rawString.IndexOf("--");
                if (getDashFromHead < 0) break;

                // index should not change as erase and addition will happen on index behind
                int wordStart = rawString.LastIndexOf(" ", getDashFromHead);
                int ssmlCheck = rawString.LastIndexOf(">", getDashFromHead); // space could be inside ssml format
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = 0; // no spaces prior should mean head of sentence
                else wordStart += 1;

                // insert end first
                int wordEnd = rawString.IndexOf(" ", getDashFromHead);
                if (wordEnd < 0) wordEnd = rawString.IndexOf(".", getDashFromHead);
                if (wordEnd < 0) wordEnd = rawString.IndexOf(",", getDashFromHead);
                if (wordEnd < 0) wordEnd = rawString.IndexOf("?", getDashFromHead);
                if (wordEnd < 0) wordEnd = rawString.IndexOf("!", getDashFromHead);
                if (wordEnd < 0) wordEnd = rawString.Length;

                rawString = rawString.Insert(wordEnd, "</prosody>");

                // erase dash dash
                rawString = rawString.Remove(getDashFromHead, 2);
                rawString = rawString.Insert(wordStart, "<prosody rate=\"x-slow\">");
            }

            // convert "..." syntax to break
            rawString.Replace("...", "<break time=\"0.5s\">");

            Prompt speech =
                new Prompt("<?xml version=\"1.0\"?><speak version=\"1.0\" xml:lang=\"" + lang + "\"><prosody volume=\"100.0\" pitch=\"+20Hz\">"
                + rawString + "</prosody></speak>", SynthesisTextFormat.Ssml);
            this.speechSynthesizer.SpeakAsync(speech);
        }

        public void SpeechEnd_FrameArrived(object sender, SpeakCompletedEventArgs e) {
            client.Publish("/kinect/state/ttsfinished", new byte[] { });
        }

        private void Close() {
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

            if (this.speechSynthesizer != null) {
                this.speechSynthesizer.Dispose();
                this.speechSynthesizer = null;
            }

            if (this.client != null) {
                this.client.Disconnect();
                this.client = null;
            }
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e) {
            Close();
        }

#if PUBLISH_RAW_AUDIO
        private void ConvertKinectAudioStream(byte[] audioIn, byte[] audioOut) {
            for (int i = 0; i < audioIn.Length / sizeof(float); ++i) {
                float sample = BitConverter.ToSingle(audioIn, i * sizeof(float));
                if (sample > 1.0f) sample = 1.0f;
                else if (sample < -1.0f) sample = -1.0f;
                short convertedSample = Convert.ToInt16(sample * short.MaxValue);
                byte[] local = BitConverter.GetBytes(convertedSample);
                System.Buffer.BlockCopy(local, 0, audioOut, i * sizeof(short), sizeof(short));
            }
        }
#endif

        private void RegisterApp_Click(object sender, RoutedEventArgs e) {
            string appKey = "kinectmicrophoneinteraction";

            using (var hkcr = Registry.ClassesRoot) {
                foreach (var key in hkcr.GetSubKeyNames())
                    if (key == appKey) {
                        this.RegisterApp.Content = "failed";
                        return;
                    }

                using (var key = hkcr.CreateSubKey(appKey)) {
                    key.SetValue(string.Empty, "Url: WPF Target Protocol");
                    key.SetValue("URL Protocol", string.Empty);
                    key.SetValue("UseOriginalUrlEncoding", 1, RegistryValueKind.DWord);
                    using (var shellKey = key.CreateSubKey("shell")) {
                        using (var openKey = shellKey.CreateSubKey("open")) {
                            using (var commandKey = openKey.CreateSubKey("command")) {
                                commandKey.SetValue(string.Empty, "\"" + Assembly.GetExecutingAssembly().Location + "\"" + "\"%1\"");
                                commandKey.Close();
                            }
                            openKey.Close();
                        }
                        shellKey.Close();
                    }
                    key.Close();
                }
                this.RegisterApp.Content = "complete";
            }
        }

    }
}
