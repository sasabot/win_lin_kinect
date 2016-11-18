#define COMPILE_SPEECH_RECOGNITION
//#define USE_KINECT_BODY_SDK

using Grpc.Core;

namespace KinectRgbdInteractionServer
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
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using System.Text;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor = null;

        private MultiSourceFrameReader multiSourceFrameReader = null;

        // bodies

        private int maxBodies = 3;
#if USE_KINECT_BODY_SDK
        private Body[] bodies = null;

        // faces

        private FaceFrameSource[] faceSource = null;
        private FaceFrameReader[] faceReader = null;
        private FaceFrameResult[] faceResult = null;
        // save face bounds for tracked-but-face-not-found case
        private RectI[] savedFaceBounds = null;

        // faces settings

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
#endif

#if COMPILE_SPEECH_RECOGNITION

        // audio

        private AudioBeamFrameReader audioReader = null;
        private readonly byte[] audioBuffer = null;
        // save buffers for mistaken audio cuts
        private List<byte[]> savedAudioBuffers;

        // speech

        private DataRecognitionClient recogClient = null;
        private bool[] speakingResult = null;
        // Speaking result is true with non-speech sounds
        // Worded result checks whether sound is speech
        private bool wordedResult = false; // TODO

        // speech settings

        private SpeechAudioFormat audioFormat;

        // speech temporary state variables

        private bool finishingSpeech = false;
        private TimeSpan pauseDetectedTime;

        // speech parameters

        // In English speech the average pause between conversations is .74 seconds
        // Therefore threshold around .74 seconds should be appropriate
        // In Japanese speech the average is 5.15 to 8.5 seconds
        private const int speechFinishThreshold = 500; // millisecs
        private const double speakerDistanceThreshold = 2.0; // meters

        // offline speech engine

        private SpeechRecognitionEngine speechEngine = null;

#endif

        // rgbd streaming

        private CoordinateMapper coordinateMapper = null;
        private int rgbdThreadCount = 0;
        private ReaderWriterLockSlim rgbdThreadLock = null;

        // rgbd streaming parameters

        public const int divideStream = 2; // if stream is small set to 1, if large set to N > 2
        public const int rgbdThreads = 2; // should limit number of threads to avoid corruption

        // grpc

        private int numClients;
        private Channel[] channel = null;
        private Kinectperson.KinectPerson.KinectPersonClient[] client = null;

        private Server server;
        private Kinectrobot.KinectRobotImpl robotImpl = null;

        // runtime settings

        private float rate = 0.2f; // this sets rgbd stream refresh rate (seconds)
        private const int frameUnit = 30;

        // runtime temporary state variables

        private int skippedFrame = 0;

        // display settings (should equal ColorFrameSource)

        public int displayWidth;
        public int displayHeight;

        // azure keys

        private string subscriptionKey;

        // runtime debug

        public TimeSpan dbgProgramStartTime;
        private int dbgFrames;
        private int dbgRgbdDroppedFrames;
        private System.Timers.Timer errTimer = null;
        private TimeSpan dbgLastDetectedTime;

        public MainWindow()
        {
            // setup Kinect

            this.kinectSensor = KinectSensor.GetDefault();

            this.multiSourceFrameReader =
#if USE_KINECT_BODY_SDK
                this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body);
#else
                this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
#endif
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_FrameArrived;

            this.displayWidth = this.kinectSensor.ColorFrameSource.FrameDescription.Width;
            this.displayHeight = this.kinectSensor.ColorFrameSource.FrameDescription.Height;

#if USE_KINECT_BODY_SDK
            // setup bodies

            this.maxBodies = this.kinectSensor.BodyFrameSource.BodyCount;
            this.bodies = new Body[this.maxBodies];

            // setup faces

            this.faceSource = new FaceFrameSource[this.maxBodies];
            this.faceReader = new FaceFrameReader[this.maxBodies];
            this.faceResult = new FaceFrameResult[this.maxBodies];
            this.savedFaceBounds = new RectI[this.maxBodies];
            
            for (int i = 0; i < this.maxBodies; ++i)
            {
                this.faceSource[i] = new FaceFrameSource(this.kinectSensor, 0, DefaultFaceFrameFeatures);
                this.faceReader[i] = this.faceSource[i].OpenReader();
                this.faceReader[i].FrameArrived += Face_FrameArrived;
            }
#endif

#if COMPILE_SPEECH_RECOGNITION

            Console.WriteLine("Enter language (options: en-US, ja-JP, ...) (press enter if same)");
            string language = Console.ReadLine().ToString();
            if (language != "")
            {
                Properties.Settings.Default.Language = language;
                Properties.Settings.Default.Save();
            }
            language = Properties.Settings.Default.Language;
            Console.WriteLine("language: {0}", language);

            Console.WriteLine("Use Kinect as audio source? (y or n)");
            string useKinectAsAudioSrcStr = Console.ReadLine().ToString();
            bool useKinectAsAudioSrc = false;
            if (useKinectAsAudioSrcStr == "y" || useKinectAsAudioSrcStr == "Y") useKinectAsAudioSrc = true;
            else Console.WriteLine("NOT using Kinect as audio source.");

            Console.WriteLine("Use Microsoft Cognitive Services (online speech recognition)? (y or n)");
            string useOnlineStr = Console.ReadLine().ToString();
            bool useOnline = false;
            if (useOnlineStr == "y" || useOnlineStr == "Y") useOnline = true;

            if (useOnline && useKinectAsAudioSrc)
            {
                Console.WriteLine("Using online recognition.");

                // setup audio

                AudioSource audioSource = this.kinectSensor.AudioSource;
                this.audioReader = audioSource.OpenReader();
                this.audioReader.FrameArrived += this.Audio_FrameArrived;

                this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];
                this.savedAudioBuffers = new List<byte[]>();

                // setup speech recognition

                this.audioFormat = new SpeechAudioFormat
                {
                    EncodingFormat = AudioCompressionType.PCM,
                    BitsPerSample = 16, // 32-bit input will be coverted to 16-bit for recognition
                    ChannelCount = 1, // only 1 channel is used
                    SamplesPerSecond = 16000,
                    AverageBytesPerSecond = 32000,
                    BlockAlign = 2 // BytesPerSample * ChannelCount
                };

                Console.WriteLine("Enter subscription key for speech recognition (press enter if same)");
                string key = Console.ReadLine().ToString();
                if (key != "")
                {
                    Properties.Settings.Default.Key = key;
                    Properties.Settings.Default.Save();
                }
                this.subscriptionKey = Properties.Settings.Default.Key;
                Console.WriteLine("got key {0}\n", this.subscriptionKey);

                this.recogClient = SpeechRecognitionServiceFactory.CreateDataClient(
                    SpeechRecognitionMode.ShortPhrase,
                    language,
                    this.subscriptionKey,
                    this.subscriptionKey);
                this.recogClient.OnResponseReceived += this.SpeechRecognition_FrameArrived;
                this.recogClient.SendAudioFormat(audioFormat);
            }
            else if (!useOnline)
            {
                Console.WriteLine("Using offline recognition. This uses template recognition.");

                // setup audio

                if (useKinectAsAudioSrc)
                {
                    AudioSource audioSource = this.kinectSensor.AudioSource;
                    this.audioReader = audioSource.OpenReader();
                    this.audioReader.FrameArrived += this.AudioOffline_FrameArrived;
                }

                // setup speech

                RecognizerInfo ri = null;
                try
                {
                    foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
                    {
                        string value;

                        if (useKinectAsAudioSrc)
                        {
                            recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                            if ("True".Equals(value, StringComparison.OrdinalIgnoreCase)
                                && language.Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                            {
                                ri = recognizer;
                                break;
                            }
                        }
                        else
                        {
                            if (language.Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                            {
                                ri = recognizer;
                                break;
                            }
                        }
                        
                    }
                }
                catch { ri = null; }

                if (null != ri)
                {
                    this.speechEngine = new SpeechRecognitionEngine(ri.Id);
                    Console.WriteLine(@"Enter grammar file. Note, all grammar files must be located in \Grammar. (press enter if same)");
                    string grammar = Console.ReadLine().ToString();
                    if (grammar != "")
                    {
                        Properties.Settings.Default.Grammar = grammar;
                        Properties.Settings.Default.Save();
                    }
                    grammar = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + @"\Grammar\" + Properties.Settings.Default.Grammar;
                    Console.WriteLine("reading grammar file: {0}", grammar);                  
                    this.speechEngine.LoadGrammar(new Grammar(grammar));

                    this.speechEngine.SpeechRecognized += this.SpeechRecognitionOffline_FrameArrived;
                    this.speechEngine.SpeechRecognitionRejected += this.SpeechRecognitionOffline_Rejected;

                    this.speechEngine.SetInputToDefaultAudioDevice();
                    this.speechEngine.RecognizeAsync(RecognizeMode.Multiple);
                }

            }

            this.speakingResult = new bool[this.maxBodies];

#endif

            // setup rgbd streaming

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // setup grpc

            Console.WriteLine("Enter number of Clients [example] 1");
            string numClients = Console.ReadLine().ToString();
            if (numClients != "")
            {
                Properties.Settings.Default.Clients = Convert.ToInt32(numClients);
                Properties.Settings.Default.Save();
            }
            this.numClients = Properties.Settings.Default.Clients;
            this.channel = new Channel[this.numClients];
            this.client = new Kinectperson.KinectPerson.KinectPersonClient[this.numClients];

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

            // setup debug (programStartTime used in Kinectrobot)
            this.dbgProgramStartTime = DateTime.Now.TimeOfDay;
            this.dbgFrames = 0;
            this.dbgRgbdDroppedFrames = 0;
            this.dbgLastDetectedTime = this.dbgProgramStartTime;

            // setup grpc: start TTS server
            string host = Dns.GetHostName();
            this.robotImpl = new Kinectrobot.KinectRobotImpl(this);
            this.server = new Server
            {
                Services = { Kinectrobot.KinectRobot.BindService(this.robotImpl) },
                Ports = { new ServerPort(Dns.GetHostByName(host).AddressList[0].ToString(), 50051, ServerCredentials.Insecure) }
            };
            this.server.Start();

            // setup grpc: start robot clients
            Kinectperson.Text thisIp = new Kinectperson.Text { Text_ = Dns.GetHostByName(host).AddressList[0].ToString() + ":50051" };
            for (int i = 0; i < this.numClients; ++i)
            {
                Kinectperson.Response response = this.client[i].CreateRobotClient(thisIp);
            }

            // setup finish

            this.kinectSensor.Open();

            // start the errTimer program manager
            this.errTimer = new System.Timers.Timer(1000);
            errTimer.Elapsed += ErrTimerEvent;
            errTimer.AutoReset = true;
            errTimer.Enabled = true;

            this.rgbdThreadLock = new ReaderWriterLockSlim();

            InitializeComponent();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
#if USE_KINECT_BODY_SDK
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
#endif

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

#if COMPILE_SPEECH_RECOGNITION
            if (this.recogClient != null)
            {
                this.recogClient.Dispose();
            }

            if (this.speechEngine != null)
            {
                this.speechEngine.SpeechRecognized -= this.SpeechRecognitionOffline_FrameArrived;
                this.speechEngine.SpeechRecognitionRejected -= this.SpeechRecognitionOffline_Rejected;
                this.speechEngine.RecognizeAsyncStop();
            }
#endif

            if (this.robotImpl != null)
            {
                this.robotImpl.Dispose();
            }
            this.server.ShutdownAsync().Wait();

            foreach (var c in this.channel)
            {
                c.ShutdownAsync().Wait();
            }
        }

        private void ErrTimerEvent(Object source, System.Timers.ElapsedEventArgs e)
        {
            // when a three second delay is detected, restart the kinect
            if ((DateTime.Now.TimeOfDay - this.dbgLastDetectedTime).Seconds > 3 && this.kinectSensor != null && this.kinectSensor.IsOpen)
            {
                LogInfo("dbgTimeElapsed", "  detected freeze restarting!", true);
                this.kinectSensor.Close();
                this.kinectSensor.Open();
            }
        }

        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // update debug info
            ++this.dbgFrames;
            LogInfo("dbgTimeElapsed",
                "passed:              " + (DateTime.Now.TimeOfDay - this.dbgProgramStartTime).Minutes
                + "min" + (DateTime.Now.TimeOfDay - this.dbgProgramStartTime).Seconds + "sec, frames: " + this.dbgFrames);
            LogInfo("dbgFailedFrames", "failed frames: " + this.dbgRgbdDroppedFrames);
            this.dbgLastDetectedTime = DateTime.Now.TimeOfDay;

            var frame = e.FrameReference.AcquireFrame();

            // person detection is real time
            Thread personThread = new Thread(() => { this.SendPersonDetectionResults(frame); });
            personThread.Start();

            // rgbd streaming is not real time
            // code might crash with 30fps, set parameter according to computer specifications
            if (this.skippedFrame < MainWindow.frameUnit * this.rate)
            {
                ++skippedFrame;
                return;
            }
            skippedFrame = 0;

            Thread rgbdThread = new Thread(() => { this.RgbdOnce(frame); });
            rgbdThread.Start();
        }

        private void SendPersonDetectionResults(MultiSourceFrame frame)
        {
            Kinectperson.PersonStream result = new Kinectperson.PersonStream { Status = -1 };

            if (frame == null)
            {
                if (this.robotImpl.GetStreamPersonSetting())
                    for (int i = 0; i < this.numClients; ++i)
                    {
                        Kinectperson.Response badresponse = this.client[i].SendPersonState(result);
                    }
                return;
            }

#if USE_KINECT_BODY_SDK
            using (var bodyFrame = frame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                {
                    if (this.robotImpl.GetStreamPersonSetting())
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
                        // below for face looking debugging
                        double pitchD, yawD;
                        pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
                        yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
                        if (pitchD > 0.0 && Math.Abs(yawD) < 20.0) LogInfo("faces", "1 ", true);
                        else if (pitchD > 0.0) LogInfo("faces", "P ", true);
                        else if (Math.Abs(yawD) < 20.0) LogInfo("faces", "Y ", true);
                        else LogInfo("faces", "0 ", true);
                    }

                    else LogInfo("faces", "0 ", true);
#if COMPILE_SPEECH_RECOGNITION
                    if (this.speakingResult[i]) LogInfo("speakers", "1 ", true);
                    else LogInfo("speakers", "0 ", true);
#endif

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
#if COMPILE_SPEECH_RECOGNITION
                                Speaking = this.speakingResult[i],
#endif
                                Looking = looking,
                                Position = new Kinectperson.Point
                                {
                                    X = this.bodies[i].Joints[JointType.Head].Position.X,
                                    Y = this.bodies[i].Joints[JointType.Head].Position.Y + Convert.ToSingle(0.1)
                                },
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
#if COMPILE_SPEECH_RECOGNITION
                                Speaking = speakingResult[i],
#endif
                                Looking = false,
                                Position = new Kinectperson.Point
                                {
                                    X = this.bodies[i].Joints[JointType.Head].Position.X,
                                    Y = this.bodies[i].Joints[JointType.Head].Position.Y + Convert.ToSingle(0.1)
                                },
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

                if (this.robotImpl.GetStreamPersonSetting())
                {
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
#endif
        }

        private void RgbdOnce(MultiSourceFrame frame)
        {
            // do not proceed if too many threads are already on-going
            this.rgbdThreadLock.EnterReadLock();
            try { if (this.rgbdThreadCount > KinectRgbdInteractionServer.MainWindow.rgbdThreads) return; }
            finally { this.rgbdThreadLock.ExitReadLock(); }

            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null || depthFrame == null)
            {
                ++this.dbgRgbdDroppedFrames;
                return;
            }

            // update running thread count
            this.rgbdThreadLock.EnterWriteLock();
            try { ++this.rgbdThreadCount; }
            finally { this.rgbdThreadLock.ExitWriteLock(); }

            // from here, rgbd process

            this.robotImpl.SetCameraInfo(this.coordinateMapper.GetDepthCameraIntrinsics());

            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            // get physical xyz position for each point on depth map
            CameraSpacePoint[] cameraPointsDepth = new CameraSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToCameraSpace(depthData, cameraPointsDepth);

            // get physical xyz position for each point on color map
            CameraSpacePoint[] cameraPointsColor = new CameraSpacePoint[colorDesc.Width * colorDesc.Height];
            this.coordinateMapper.MapColorFrameToCameraSpace(depthData, cameraPointsColor);

            // get corresponding color pixel for each point on depth map
            ColorSpacePoint[] colorPoints = new ColorSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToColorSpace(depthData, colorPoints);

            int pointIndex = 0;
            if (this.robotImpl.GetStreamRgbdSettings())
            {
                Kinectperson.PointStream pointStream = new Kinectperson.PointStream { };
                foreach (CameraSpacePoint point in cameraPointsDepth)
                {
                    // get color from corresponding color pixel
                    // set default values
                    int color = ((255 << 16) & 0xfffffff) + ((255 << 8) & 0xfffffff) + 255;
                    int img_y = -1;
                    int img_x = -1;
                    // below check required if point with infinity or nan value is not rejected beforehand
                    if (!Double.IsInfinity(colorPoints[pointIndex].X) && !Double.IsInfinity(colorPoints[pointIndex].Y)
                        && !Double.IsNaN(colorPoints[pointIndex].X) && !Double.IsNaN(colorPoints[pointIndex].Y))
                    {
                        img_y = Convert.ToInt32(colorPoints[pointIndex].Y);
                        img_x = Convert.ToInt32(colorPoints[pointIndex].X);
                    }
                    // note, corresponding pixel can be out of range on color map, due to coordinate difference
                    if (img_x >= 0 && img_y >= 0 && img_x < colorDesc.Width && img_y < colorDesc.Height)
                    {
                        int pixel = 4 * (img_y * colorDesc.Width + img_x); // bgra, so skip by 4
                        color = ((pixels[pixel++] << 16) & 0xfffffff) + ((pixels[pixel++] << 8) & 0xfffffff) + pixels[pixel++];
                    }

                    // add point to stream
                    Kinectperson.Rgbd p = new Kinectperson.Rgbd
                    {
                        Color = color,
                        X = point.X,
                        Y = point.Y,
                        Z = point.Z
                    };
                    ++pointIndex;
                    pointStream.Data.Add(p);
                }

                Task.Run(() => SendPointStream(pointStream));

                colorFrame.Dispose();
                depthFrame.Dispose();

                // release this thread
                this.rgbdThreadLock.EnterWriteLock();
                try { --this.rgbdThreadCount; }
                finally { this.rgbdThreadLock.ExitWriteLock(); }

                return;
            }

            Kinectrobot.Points points = new Kinectrobot.Points { };
            foreach (CameraSpacePoint point in cameraPointsDepth)
            {
                // get color from corresponding color pixel
                // set default values
                int color = ((255 << 16) & 0xfffffff) + ((255 << 8) & 0xfffffff) + 255;
                int img_y = -1;
                int img_x = -1;
                // below check required if point with infinity or nan value is not rejected beforehand
                if (!Double.IsInfinity(colorPoints[pointIndex].X) && !Double.IsInfinity(colorPoints[pointIndex].Y)
                    && !Double.IsNaN(colorPoints[pointIndex].X) && !Double.IsNaN(colorPoints[pointIndex].Y))
                {
                    img_y = Convert.ToInt32(colorPoints[pointIndex].Y);
                    img_x = Convert.ToInt32(colorPoints[pointIndex].X);
                }
                // note, corresponding pixel can be out of range on color map, due to coordinate difference
                if (img_x >= 0 && img_y >= 0 && img_x < colorDesc.Width && img_y < colorDesc.Height)
                {
                    int pixel = 4 * (img_y * colorDesc.Width + img_x); // bgra, so skip by 4
                    color = ((pixels[pixel++] << 16) & 0xfffffff) + ((pixels[pixel++] << 8) & 0xfffffff) + pixels[pixel++];
                }

                // add point to stream
                Kinectrobot.Point p = new Kinectrobot.Point
                {
                    Color = color,
                    X = point.X,
                    Y = point.Y,
                    Z = point.Z
                };
                ++pointIndex;
                points.Data.Add(p);
            }

            this.robotImpl.SetPoints(points, colorPoints);

            int pixelIndex = 0;          
            Kinectrobot.Pixels image = new Kinectrobot.Pixels { };
            while (true)
            {
                // get color
                int pixel = 4 * pixelIndex; // bgra, so skip by 4
                if (pixel >= pixels.Length) break;

                int color = ((pixels[pixel++] << 16) & 0xfffffff) + ((pixels[pixel++] << 8) & 0xfffffff) + pixels[pixel++];

                // add pixel to stream
                ++pixelIndex;
                image.Color.Add(color);
            }

            this.robotImpl.SetPixels(image, pixels, cameraPointsColor);

            colorFrame.Dispose();
            depthFrame.Dispose();

            // release this thread
            this.rgbdThreadLock.EnterWriteLock();
            try { --this.rgbdThreadCount; }
            finally { this.rgbdThreadLock.ExitWriteLock(); }
        }

        private async Task SendPointStream(Kinectperson.PointStream points)
        {
            Kinectperson.PointStream pointStream = new Kinectperson.PointStream { };
            int pointsPerStream = points.Data.Count / KinectRgbdInteractionServer.MainWindow.divideStream + 1;

            for (int k = 0; k < this.numClients; ++k)
            {
                try
                {
                    using (var send = this.client[k].SendPointStream())
                    {
                        for (int i = 0; i < KinectRgbdInteractionServer.MainWindow.divideStream; ++i)
                        {
                            Kinectperson.PointStream blob = new Kinectperson.PointStream { };
                            int range = System.Math.Min((i + 1) * pointsPerStream, points.Data.Count);
                            for (int j = pointsPerStream * i; j < range; ++j)
                                blob.Data.Add(points.Data[j]);
                            await send.RequestStream.WriteAsync(blob);
                        }
                        await send.RequestStream.CompleteAsync();
                        Kinectperson.Response response = await send.ResponseAsync;
                    }
                }
                catch (RpcException e)
                {
                    throw;
                }
            }
        }

#if USE_KINECT_BODY_SDK
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
#endif

#if COMPILE_SPEECH_RECOGNITION

        private void Audio_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            if (this.robotImpl.RecognitionOffTriggerOn()) return; // don't get voice while robot is talking
            // the above is also a mutex safe trigger when speech recognition analysis is going on

            AudioBeamFrameReference frameReference = e.FrameReference;
            using (var frameList = frameReference.AcquireBeamFrames())
            {
                if (frameList == null) return;

                IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

#if USE_KINECT_BODY_SDK
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
#endif

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
                            > KinectRgbdInteractionServer.MainWindow.speechFinishThreshold)
                        {
                            // end voice collect
                            this.recogClient.EndAudio();
                            return; // wait for result in SpeechRecognition_FrameArrived
                        }
                        LogInfo("audio", "Delaying " + (DateTime.Now.TimeOfDay - this.pauseDetectedTime).Milliseconds);
                    }
                    else
                    {
                        // extra voice detected, so escape finishingSpeech mode
                        this.finishingSpeech = false;
                    }

                    // send voice if speech
                    subFrame.CopyFrameDataToArray(this.audioBuffer);
                    byte[] convertedBuffer = new byte[Convert.ToInt32(this.audioBuffer.Length * 0.5)];
                    ConvertKinectAudioStream(this.audioBuffer, convertedBuffer);
                    this.recogClient.SendAudio(convertedBuffer, convertedBuffer.Length);
                    // save voice to buffer (see SpeechRecognition_FrameArrived for details)
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

        private void SpeechRecognition_FrameArrived(object sender, SpeechResponseEventArgs e)
        {
            // if not a speech or voice, return
            if (e.PhraseResponse.Results.Length == 0)
            {
                // display log
                LogInfo("status", "detected noise " + DateTime.Now.TimeOfDay);
                LogInfo("recognized", "");

                // lock recogClient from being overwritten by voice capture
                this.robotImpl.RecognitionOff();

                // add in case partial speech is mistaken as noise
                for (int i = 0; i < this.savedAudioBuffers.Count; ++i)
                {
                    this.recogClient.SendAudio(this.savedAudioBuffers[i], this.savedAudioBuffers[i].Length);
                }
                this.savedAudioBuffers.Clear();

                // free lock
                this.robotImpl.RecognitionOn();

                return;
            }

            // valid speech, clear buffer
            this.savedAudioBuffers.Clear();

            // display log
            LogInfo("status", "recognized speech");
            LogInfo("recognized", e.PhraseResponse.Results[0].DisplayText);

            // close capturing voice until robot has finished talking
            if (this.robotImpl.triggerAfterRecognition) this.robotImpl.RecognitionOff();

            Task.Run(() => SendSpeechRecognitionResults(e.PhraseResponse.Results[0].DisplayText));  
        }

        private void AudioOffline_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference frameReference = e.FrameReference;
            using (var frameList = frameReference.AcquireBeamFrames())
            {
                if (frameList == null) return;

                IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

#if USE_KINECT_BODY_SDK
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
#endif
            }
        }

        private void SpeechRecognitionOffline_FrameArrived(object sender, SpeechRecognizedEventArgs e)
        {
            if (this.robotImpl.RecognitionOffTriggerOn())
            {
                // display log
                LogInfo("status", "speech recognition is not sended");
                LogInfo("audio", e.Result.Semantics.Value.ToString());
                return; // don't get voice while robot is talking
            }
            // the above is also a mutex safe trigger when speech recognition analysis is going on

            const double ConfidenceThreshold = 0.3;

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                // display log
                LogInfo("status", "recognized speech");
                LogInfo("audio", e.Result.Semantics.Value.ToString());

                // close capturing voice until robot has finished talking
                if (this.robotImpl.triggerAfterRecognition) this.robotImpl.RecognitionOff();

                Task.Run(() => SendSpeechRecognitionResults(e.Result.Semantics.Value.ToString()));
            }
            else
            {
                // display log
                LogInfo("status", "detected but not confident");
                LogInfo("audio", e.Result.Semantics.Value.ToString());
            }
        }

        private void SpeechRecognitionOffline_Rejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            // display log
            LogInfo("status", "detected no match");
            LogInfo("audio", "");
        }

        private async Task SendSpeechRecognitionResults(string s)
        {
            List<Task<Kinectperson.Response>> tasks = new List<Task<Kinectperson.Response>>();

            // wait until all robots have finished talking
            Kinectperson.Text data = new Kinectperson.Text {  Text_ = s };
            for (int i = 0; i < this.numClients; ++i)
            {
                tasks.Add(SendSpeechRecognitionResultToClienti(i, data));
            }
            await Task.WhenAny(Task.WhenAll(tasks), Task.Delay(120000)); // wait at longest 2 minutes

            // re-open capturing voice when robot has finished talking
            if (this.robotImpl.triggerAfterRecognition) this.robotImpl.RecognitionOn();

            // flush online recognition result to show that speech recognition mode is on again
            // LogInfo("status", "starting recognition");
            LogInfo("recognized", "");
        }

        private async Task<Kinectperson.Response> SendSpeechRecognitionResultToClienti(int i, Kinectperson.Text data)
        {
            // response will not come back until robot side says okay
            Kinectperson.Response response = this.client[i].SendVoiceRecognition(data);
            return response;
        }

#endif

        public void LogInfo(string blockName, string text, bool concatenate=false)
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
