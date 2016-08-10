using Grpc.Core;

namespace KinectSimpleInteractionServer
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Face;
    using System.Linq;
    using System.Web;
    using System.IO;
    using System.Threading.Tasks;
    using System.Net.Http;
    using System.Net.Http.Headers;
    using Newtonsoft.Json.Linq;
    using Microsoft.ProjectOxford.SpeechRecognition;
    using System.Threading;
    using System.Windows.Controls;
    using System.Net;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor = null;

        private MultiSourceFrameReader multiSourceFrameReader = null;

        private FaceFrameSource[] faceSource = null;
        private FaceFrameReader[] faceReader = null;
        private Body[] bodies = null;

        private const FaceFrameFeatures DefaultFaceFrameFeatures =
            FaceFrameFeatures.BoundingBoxInColorSpace
            | FaceFrameFeatures.PointsInColorSpace
            | FaceFrameFeatures.Happy
            | FaceFrameFeatures.FaceEngagement
            | FaceFrameFeatures.Glasses
            | FaceFrameFeatures.LeftEyeClosed
            | FaceFrameFeatures.RightEyeClosed
            | FaceFrameFeatures.MouthOpen
            | FaceFrameFeatures.MouthMoved
            | FaceFrameFeatures.LookingAway
            | FaceFrameFeatures.RotationOrientation;

        private FaceFrameResult[] faceResult = null;
        private RectI[] savedFaceBounds = null;
        
        private int maxBodies;

        private int displayWidth;

        private int displayHeight;

        private AudioBeamFrameReader audioReader = null;
        private bool[] speakingResult = null;

        // Speaking result is true with non-speech sounds
        // Worded result checks whether sound is speech
        private bool wordedResult = false;

        private DataRecognitionClient recogClient;
        private string subscriptionKey;
        private SpeechAudioFormat audioFormat;
        private readonly byte[] audioBuffer = null;
        private List<byte[]> savedAudioBuffers;

        // In English speech the average pause between conversations is .74 seconds
        // Therefore threshold around .74 seconds should be appropriate
        // In Japanese speech the average is 5.15 to 8.5 seconds
        private const int speechFinishThreshold = 500; // millisecs
        private bool finishingSpeech = false;
        private TimeSpan pauseDetectedTime;

        private const double speakerDistanceThreshold = 2.0; // meters

        private Channel[] channel = null;

        private Kinectperson.KinectPerson.KinectPersonClient[] client = null;

        private int numClients;

        private Server server;

        private Kinectrobot.KinectRobotImpl robotImpl = null;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.multiSourceFrameReader =
                this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_FrameArrived;

            // setup for face detection
            this.maxBodies = this.kinectSensor.BodyFrameSource.BodyCount;
            this.faceSource = new FaceFrameSource[this.maxBodies];
            this.faceReader = new FaceFrameReader[this.maxBodies];
            this.faceResult = new FaceFrameResult[this.maxBodies];
            // save face bounds for tracked-but-face-not-found case
            this.savedFaceBounds = new RectI[this.maxBodies];
            this.bodies = new Body[this.maxBodies];
            for (int i = 0; i < this.maxBodies; ++i)
            {
                this.faceSource[i] = new FaceFrameSource(kinectSensor, 0, DefaultFaceFrameFeatures);
                this.faceReader[i] = this.faceSource[i].OpenReader();
                this.faceReader[i].FrameArrived += Face_FrameArrived;
            }

            this.displayWidth = this.kinectSensor.ColorFrameSource.FrameDescription.Width;
            this.displayHeight = this.kinectSensor.ColorFrameSource.FrameDescription.Height;

            this.kinectSensor.Open();

            // setup for audio detection
            AudioSource audioSource = this.kinectSensor.AudioSource;
            this.audioReader = audioSource.OpenReader();
            this.audioReader.FrameArrived += this.Audio_FrameArrived;
            this.speakingResult = new bool[this.maxBodies];

            // setup TTS server
            string host = Dns.GetHostName();
            this.robotImpl = new Kinectrobot.KinectRobotImpl(this.LogInfo);
            this.server = new Server
            {
                Services = { Kinectrobot.KinectRobot.BindService(this.robotImpl) },
                Ports = { new ServerPort(Dns.GetHostByName(host).AddressList[0].ToString(), 50051, ServerCredentials.Insecure) }
            };
            this.server.Start();

            // setting up communications

            Console.WriteLine("Enter number of Clients [example] 1");
            string numClients = Console.ReadLine().ToString();
            if (numClients != "")
            {
                Properties.Settings.Default.Clients = Convert.ToInt32(numClients);
                Properties.Settings.Default.Save();
            }
            this.numClients = Properties.Settings.Default.Clients;
            this.client = new Kinectperson.KinectPerson.KinectPersonClient[this.numClients];
            this.channel = new Channel[this.numClients];

            if (Properties.Settings.Default.Ips == null)
                Properties.Settings.Default.Ips = new System.Collections.Specialized.StringCollection { };

            for (int i = 0; i < System.Math.Min(Properties.Settings.Default.Ips.Count, this.numClients); ++i)
            {
                Console.WriteLine("Enter IP for changes from {0} (press enter if no)", Properties.Settings.Default.Ips[i]);
                string ip = Console.ReadLine().ToString();
                if (ip != "")
                {
                    Properties.Settings.Default.Ips.Insert(i, ip);
                    Properties.Settings.Default.Save();
                }
                this.channel[i] = new Channel(Properties.Settings.Default.Ips[i], ChannelCredentials.Insecure);
                this.client[i] = new Kinectperson.KinectPerson.KinectPersonClient(this.channel[i]);
            }

            int count = Properties.Settings.Default.Ips.Count;
            for (int i = count; i < this.numClients; ++i)
            {
                Console.WriteLine("Add new IP [example] 10.172.99.162:50052");
                string ip = Console.ReadLine().ToString();
                Properties.Settings.Default.Ips.Insert(i, ip);
                Properties.Settings.Default.Save();
                this.channel[i] = new Channel(Properties.Settings.Default.Ips[i], ChannelCredentials.Insecure);
                this.client[i] = new Kinectperson.KinectPerson.KinectPersonClient(this.channel[i]);
            }

            // setup speech recognition
            Console.WriteLine("Enter subscription key for voice recognition (press enter if same)");
            string key = Console.ReadLine().ToString();
            if (key != "")
            {
                Properties.Settings.Default.Key = key;
                Properties.Settings.Default.Save();
            }
            this.subscriptionKey = Properties.Settings.Default.Key;
            Console.WriteLine("got key {0}", this.subscriptionKey);

            this.audioFormat = new SpeechAudioFormat
            {
                EncodingFormat = AudioCompressionType.PCM,
                BitsPerSample = 16, // 32-bit input will be coverted to 16-bit for recognition
                ChannelCount = 1, // only 1 channel is used
                SamplesPerSecond = 16000,
                AverageBytesPerSecond = 32000,
                BlockAlign = 2 // BytesPerSample * ChannelCount
            };

            this.recogClient = SpeechRecognitionServiceFactory.CreateDataClient(
                SpeechRecognitionMode.ShortPhrase,
                "en-US",
                this.subscriptionKey,
                this.subscriptionKey);
            this.recogClient.OnResponseReceived += this.VoiceRecognition_FrameArrived;
            this.recogClient.SendAudioFormat(audioFormat);

            this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];

            // save buffers for mistaken audio cuts
            this.savedAudioBuffers = new List<byte[]>();

            // start robot clients
            Kinectperson.Text thisIp = new Kinectperson.Text { Text_ = Dns.GetHostByName(host).AddressList[0].ToString() + ":50051" };
            for (int i = 0; i < this.numClients; ++i)
            {
                Kinectperson.Response response = this.client[i].CreateRobotClient(thisIp);
            }

            InitializeComponent();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            for (int i = 0; i < this.maxBodies; ++i)
            {
                if (this.faceReader[i] != null)
                {
                    this.faceReader[i].Dispose();
                    this.faceReader[i] = null;
                }

                if (this.faceSource[i] != null)
                {
                    this.faceSource[i].Dispose();
                    this.faceSource[i] = null;
                }
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            if (this.recogClient != null)
            {
                this.recogClient.Dispose();
            }

            if (this.robotImpl != null)
            {
                this.robotImpl.Dispose();
            }
            this.server.ShutdownAsync().Wait();
        }

        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var frame = e.FrameReference.AcquireFrame();
            SendPersonDetectionResults(frame);   
        }

        private async void SendPersonDetectionResults(MultiSourceFrame frame)
        {
            Kinectperson.PersonStream result = new Kinectperson.PersonStream { Status = -1 };

            if (frame == null)
            {
                for (int i = 0; i < this.numClients; ++i)
                {
                    Kinectperson.Response badresponse = this.client[i].SendPersonState(result);
                }
                return;
            }

            using (var bodyFrame = frame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                {
                    for (int i = 0; i < this.numClients; ++i)
                    {
                        Kinectperson.Response badresponse = this.client[i].SendPersonState(result);
                    }
                    return;
                }

                bodyFrame.GetAndRefreshBodyData(this.bodies);

                // display tracked bodies
                LogInfo("skeletons", "detected: ");
                LogInfo("faces", "looking:  ");
                LogInfo("speakers", "speaking: ");

                for (int i = 0; i < this.maxBodies; ++i)
                {
                    // display track info
                    if (this.bodies[i].IsTracked) LogInfo("skeletons", "1 ", true);
                    else LogInfo("skeletons", "0 ", true);
                    //if (this.faceResult[i] != null &&
                    //    this.faceResult[i].FaceProperties[FaceProperty.LookingAway] == DetectionResult.No) LogInfo("faces", "1 ", true);

                    if (this.faceResult[i] != null)
                    {
                        var faceOrientation = this.faceResult[i].FaceRotationQuaternion;
                        double x = faceOrientation.X;
                        double y = faceOrientation.Y;
                        double z = faceOrientation.Z;
                        double w = faceOrientation.W;
                        // convert face rotation quaternion to Euler angles in degrees
                        double yawD;
                        yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
                        if (Math.Abs(yawD) < 20.0) LogInfo("faces", "1 ", true);
                        else LogInfo("faces", "0 ", true);
                    }

                    else LogInfo("faces", "0 ", true);
                    if (this.speakingResult[i]) LogInfo("speakers", "1 ", true);
                    else LogInfo("speakers", "0 ", true);

                    if (this.faceSource[i].IsTrackingIdValid)
                    {
                        if (this.faceResult[i] != null)
                        {
                            var faceBounds = this.faceResult[i].FaceBoundingBoxInColorSpace;
                            var faceOrientation = this.faceResult[i].FaceRotationQuaternion;
                            double x = faceOrientation.X;
                            double y = faceOrientation.Y;
                            double z = faceOrientation.Z;
                            double w = faceOrientation.W;
                            // convert face rotation quaternion to Euler angles in degrees
                            double yawD, pitchD, rollD;
                            pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
                            yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
                            rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;
                            // clamp the values to a multiple of the specified increment to control the refresh rate
                            double increment = 5.0;
                            bool looking = true;
                            if (this.faceResult[i].FaceProperties[FaceProperty.LookingAway] == DetectionResult.Yes) looking = false;
                            Kinectperson.Person p = new Kinectperson.Person
                            {
                                Face = new Kinectperson.Face
                                {
                                    X = faceBounds.Left,
                                    Y = faceBounds.Top,
                                    Width = faceBounds.Right - faceBounds.Left,
                                    Height = faceBounds.Bottom - faceBounds.Top,
                                    Roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment),
                                    Pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment),
                                    Yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment)
                                },
                                Speaking = this.speakingResult[i],
                                Looking = looking,
                                Distance = this.bodies[i].Joints[JointType.Head].Position.Z,
                                Id = i
                            };
                            result.Data.Add(p);
                            // save face bounds for tracked-but-face-not-found case
                            this.savedFaceBounds[i] = faceBounds;
                        }
                        else
                        {
                            Kinectperson.Person p = new Kinectperson.Person
                            {
                                Face = new Kinectperson.Face
                                {
                                    X = this.savedFaceBounds[i].Left,
                                    Y = this.savedFaceBounds[i].Top,
                                    Width = this.savedFaceBounds[i].Right - this.savedFaceBounds[i].Left,
                                    Height = this.savedFaceBounds[i].Bottom - this.savedFaceBounds[i].Top,
                                    Roll = Single.NaN,
                                    Pitch = Single.NaN,
                                    Yaw = Single.NaN
                                },
                                Speaking = speakingResult[i],
                                Looking = false,
                                Distance = this.bodies[i].Joints[JointType.Head].Position.Z,
                                Id = i
                            };
                            result.Data.Add(p);
                        }
                    }
                    else
                    {
                        if (this.bodies[i].IsTracked)
                            this.faceSource[i].TrackingId = this.bodies[i].TrackingId;
                    }
                }

                // if no face found
                if (result.Data.Count == 0)
                {
                    // you could check for person existance using Microsoft Cognitive Service
                    result.Status = 0;
                    for (int i = 0; i < this.numClients; ++i)
                    {
                        Kinectperson.Response response = this.client[i].SendPersonState(result);
                    }
                    return;
                }

                result.Status = 1;

                // send results if faces are found
                for (int i = 0; i < this.numClients; ++i)
                {
                    Kinectperson.Response response = this.client[i].SendPersonState(result);
                }
            }
        }

        private void Face_FrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            using (var faceFrame = e.FrameReference.AcquireFrame())
            {
                if (faceFrame == null) return;

                // find corresponding face id
                int index = -1;
                for (int i = 0; i < this.maxBodies; ++i)
                    if (this.faceSource[i] == faceFrame.FaceFrameSource)
                    {
                        index = i;
                        break;
                    }

                var result = faceFrame.FaceFrameResult;
                if (result == null)
                {
                    this.faceResult[index] = null;
                    return;
                }

                // make sure face is within image
                var faceBox = result.FaceBoundingBoxInColorSpace;
                bool isFaceValid = (faceBox.Right - faceBox.Left) > 0 && (faceBox.Bottom - faceBox.Top) > 0 &&
                    faceBox.Right <= this.displayWidth && faceBox.Bottom <= this.displayHeight;

                if (!isFaceValid)
                {
                    this.faceResult[index] = null;
                    return;
                }

                this.faceResult[index] = result;
            }
        }

        private void Audio_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            if (this.robotImpl.RecognitionOffTriggerOn()) return; // don't get voice while robot is talking
            // the above is also a mutex safe trigger when voice recognition analysis is going on

            AudioBeamFrameReference frameReference = e.FrameReference;
            using (var frameList = frameReference.AcquireBeamFrames())
            {
                if (frameList == null) return;

                IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                // get id of person speaking
                ulong audioTrackingId = Convert.ToUInt64(this.kinectSensor.BodyFrameSource.BodyCount) + 1;
                if (subFrameList[0].AudioBodyCorrelations.Count > 0)
                {
                    audioTrackingId = subFrameList[0].AudioBodyCorrelations[0].BodyTrackingId;
                }

                // set speaker result
                for (int i = 0; i < this.faceSource.Length; ++i)
                {
                    if (this.faceSource[i].TrackingId == audioTrackingId)
                        this.speakingResult[i] = true;
                    else
                        this.speakingResult[i] = false;
                }

                // get voice
                foreach (AudioBeamSubFrame subFrame in subFrameList)
                {
                    // no voice
                    if (subFrame.BeamAngleConfidence < 0.05)
                    {
                        // when silence is detected for first time, speech recognition will start preparing speech finish
                        if (!this.finishingSpeech)
                        {
                            this.finishingSpeech = true;
                            this.pauseDetectedTime = DateTime.Now.TimeOfDay;
                        }
                        // when a long silence is detected, end speech recognition
                        else if ((DateTime.Now.TimeOfDay - this.pauseDetectedTime).Milliseconds
                            > KinectSimpleInteractionServer.MainWindow.speechFinishThreshold)
                        {
                            // end voice collect
                            this.recogClient.EndAudio();
                            return; // wait for result in VoiceRecognition_FrameArrived
                        }
                        LogInfo("audio", "Delaying " + (DateTime.Now.TimeOfDay - this.pauseDetectedTime).Milliseconds);
                    }
                    else
                    {
                        // extra voice detected, so escape finishingSpeech mode
                        this.finishingSpeech = false;
                    }

                    // send voice if voice
                    subFrame.CopyFrameDataToArray(this.audioBuffer);
                    byte[] convertedBuffer = new byte[Convert.ToInt32(this.audioBuffer.Length * 0.5)];
                    ConvertKinectAudioStream(this.audioBuffer, convertedBuffer);
                    this.recogClient.SendAudio(convertedBuffer, convertedBuffer.Length);
                    // save voice to buffer (see VoiceRecognition_FrameArrived for details)
                    this.savedAudioBuffers.Add(convertedBuffer);
                }
            }
        }

        private void ConvertKinectAudioStream(byte[] audioIn, byte[] audioOut)
        {
            // convert each float audio sample to short
            for (int i = 0; i < audioIn.Length / sizeof(float); ++i)
            {
                // extract a single 32-bit IEEE value from the byte array
                float sample = BitConverter.ToSingle(audioIn, i * sizeof(float));

                // make sure it is in the range [-1, +1]
                if (sample > 1.0f) sample = 1.0f;
                else if (sample < -1.0f) sample = -1.0f;

                // scale float to the range (short.MinValue, short.MaxValue] and then 
                // convert to 16-bit signed with proper rounding
                short convertedSample = Convert.ToInt16(sample * short.MaxValue);

                // place the resulting 16-bit sample in the output byte array
                byte[] local = BitConverter.GetBytes(convertedSample);
                System.Buffer.BlockCopy(local, 0, audioOut, i * sizeof(short), sizeof(short));
            }
        }

        private void VoiceRecognition_FrameArrived(object sender, SpeechResponseEventArgs e)
        {
            // if not a voice, return
            if (e.PhraseResponse.Results.Length == 0)
            {
                // display log
                LogInfo("status", "detected noise " + DateTime.Now.TimeOfDay);
                LogInfo("recognized", "");

                // lock recogClient from being overwritten by voice capture
                this.robotImpl.RecognitionOff();

                // add in case partial voice is mistaken as noise
                for (int i = 0; i < this.savedAudioBuffers.Count; ++i)
                {
                    this.recogClient.SendAudio(this.savedAudioBuffers[i], this.savedAudioBuffers[i].Length);
                }
                this.savedAudioBuffers.Clear();

                // free lock
                this.robotImpl.RecognitionOn();

                return;
            }

            // valid voice, clear buffer
            this.savedAudioBuffers.Clear();

            // display log
            LogInfo("status", "recognized voice");
            LogInfo("recognized", e.PhraseResponse.Results[0].DisplayText);

            // close capturing voice until robot has finished talking
            if (this.robotImpl.triggerAfterRecognition) this.robotImpl.RecognitionOff();

            Task.Run(() => SendVoiceRecognitionResults(e));  
        }

        private async Task SendVoiceRecognitionResults(SpeechResponseEventArgs e)
        {
            List<Task<Kinectperson.Response>> tasks = new List<Task<Kinectperson.Response>>();

            // wait until all robots have finished talking
            Kinectperson.Text data = new Kinectperson.Text {  Text_ = e.PhraseResponse.Results[0].DisplayText };
            for (int i = 0; i < this.numClients; ++i)
            {
                tasks.Add(SendVoiceRecognitionResultToClienti(i, data));
            }
            await Task.WhenAny(Task.WhenAll(tasks), Task.Delay(120000)); // wait at longest 2 minutes

            // re-open capturing voice when robot has finished talking
            if (this.robotImpl.triggerAfterRecognition) this.robotImpl.RecognitionOn();

            // show that voice recognition mode is on again
            LogInfo("status", "starting recognition");
            LogInfo("recognized", "");
        }

        private async Task<Kinectperson.Response> SendVoiceRecognitionResultToClienti(int i, Kinectperson.Text data)
        {
            // response will not come back until robot side says okay
            Kinectperson.Response response = this.client[i].SendVoiceRecognition(data);
            return response;
        }

        private void LogInfo(string blockName, string text, bool concatenate=false)
        {
            this.Dispatcher.Invoke((Action)(() =>
            {
                TextBlock textBlock = (TextBlock)this.canvas.FindName(blockName);
                if (textBlock != null)
                {
                    if (concatenate) textBlock.Text += text;
                    else textBlock.Text = text;
                }
            }));
        }

    }
}
