using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Core.Utils;
using System.Speech.Synthesis;
using System.Threading;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Web;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Newtonsoft.Json.Linq;
using Microsoft.Kinect;


namespace Kinectrobot
{
    class KinectRobotImpl : KinectRobot.KinectRobotBase
    {
        // rgbd streaming

        private ReaderWriterLockSlim pointsLocker = null;
        private Kinectrobot.Points[] pointBlobs = null;
        private ColorSpacePoint[] colorPoints = null; // recorded w/ depth, used for getting pixel bounds

        private ReaderWriterLockSlim pixelsLocker = null;
        private Kinectrobot.Pixels[] pixelBlobs = null;
        private byte[] rawImage = null; // recorded w/ image, used for cognition
        private CameraSpacePoint[] cameraPointsColor = null; // recorded w/ image, used for cognition

        // rgbd streaming parameters

        private int sendStartPointX = 0;
        private int sendStartPointY = 0;
        private int sendPointWidth = 512;
        private int sendPointHeight = 424;

        // speech

        private SpeechSynthesizer speechSynthesizer;

        private ReaderWriterLockSlim locker = null;
        private bool abortSpeech = false;

        public bool triggerAfterRecognition = false;
        private bool triggerDuringSpeech = false;

        // logs

        public delegate void LogInfo(string blockName, string text, bool concatenate = false);
        private LogInfo logInfo = null;

        public KinectRobotImpl(KinectRgbdInteractionServer.MainWindow parent)
        {
            // create lockers
            this.pointsLocker = new ReaderWriterLockSlim();
            this.pixelsLocker = new ReaderWriterLockSlim();
            this.locker = new ReaderWriterLockSlim();

            // setup for logs
            this.logInfo = new LogInfo(parent.LogInfo);

            // setup for rgbd streaming
            this.pointBlobs = new Kinectrobot.Points[KinectRgbdInteractionServer.MainWindow.divideStream];
            for (int i = 0; i < this.pointBlobs.Length; ++i)
            {
                this.pointBlobs[i] = new Kinectrobot.Points { };
            }

            this.pixelBlobs = new Kinectrobot.Pixels[KinectRgbdInteractionServer.MainWindow.divideStream];
            for (int i = 0; i < this.pixelBlobs.Length; ++i)
            {
                this.pixelBlobs[i] = new Kinectrobot.Pixels { };
            }

            this.colorPoints = Enumerable.Repeat(new ColorSpacePoint { X = -1, Y = -1 }, parent.displayWidth * parent.displayHeight).ToArray();
            this.rawImage = Enumerable.Repeat(new byte { }, parent.displayWidth * parent.displayHeight).ToArray();

            // setup for TTS
            this.speechSynthesizer = new SpeechSynthesizer();
            this.speechSynthesizer.SelectVoiceByHints(VoiceGender.Female, VoiceAge.Teen);
            this.speechSynthesizer.SetOutputToDefaultAudioDevice();
            this.speechSynthesizer.SpeakCompleted += this.SpeechEnd_FrameArrived;
        }

        public void Dispose()
        {
            if (this.speechSynthesizer != null)
            {
                this.speechSynthesizer.Dispose();
            }
        }

        public void SetPoints(Kinectrobot.Points points, ColorSpacePoint[] image)
        {
            int pointsPerStream = points.Data.Count / this.pointBlobs.Length + 1;

            this.pointsLocker.EnterWriteLock();
            try
            {
                Array.Copy(image, this.colorPoints, image.Length);

                for (int i = 0; i < this.pointBlobs.Length; ++i)
                {
                    this.pointBlobs[i].Data.Clear();
                    int range = System.Math.Min((i + 1) * pointsPerStream, points.Data.Count);
                    for (int j = pointsPerStream * i; j < range; ++j)
                        this.pointBlobs[i].Data.Add(points.Data[j]);
                }
            }
            finally { this.pointsLocker.ExitWriteLock(); }
        }

        public void SetPixels(Kinectrobot.Pixels pixels, byte[] raw, CameraSpacePoint[] positions)
        {
            int pointsPerStream = pixels.Color.Count / this.pixelBlobs.Length + 1;

            this.pixelsLocker.EnterWriteLock();
            try
            {
                Array.Copy(raw, this.rawImage, raw.Length);
                Array.Copy(positions, this.cameraPointsColor, positions.Length);

                for (int i = 0; i < this.pixelBlobs.Length; ++i)
                {
                    this.pixelBlobs[i].Color.Clear();
                    int range = System.Math.Min((i + 1) * pointsPerStream, pixels.Color.Count);
                    for (int j = pointsPerStream * i; j < range; ++j)
                        this.pixelBlobs[i].Color.Add(pixels.Color[j]);
                }
            }
            finally { this.pixelsLocker.ExitReadLock(); }
        }

        public override async Task ReturnPoints(Request request, IServerStreamWriter<Points> responseStream, ServerCallContext context)
        {
            this.pointsLocker.EnterReadLock();
            try
            {
                foreach (var blob in this.pointBlobs)
                {
                    await responseStream.WriteAsync(blob);
                }
            }
            finally { this.pointsLocker.ExitReadLock(); }
        }

        public override async Task ReturnImage(Request request, IServerStreamWriter<Pixels> responseStream, ServerCallContext context)
        {
            this.pixelsLocker.EnterReadLock();
            try
            {
                foreach (var blob in this.pixelBlobs)
                {
                    await responseStream.WriteAsync(blob);
                }
            }
            finally { this.pixelsLocker.ExitReadLock(); }
        }

        public override Task<BitStream> ReturnPixelBoundsFromSpaceBounds(Request request, ServerCallContext context)
        {
            this.pointsLocker.EnterReadLock();
            try
            {
                Kinectrobot.BitStream result = new Kinectrobot.BitStream { Status = false };

                foreach (var req in request.Data)
                {
                    List<int> boundsInLinux = new List<int> { Convert.ToInt32(req.X), Convert.ToInt32(req.Y),
                    Convert.ToInt32(req.Width), Convert.ToInt32(req.Height) };
                    List<int> boundsInWindows = new List<int> { };

                    // in case point cloud bounds had a region cut
                    foreach (var depthLinuxIndex in boundsInLinux)
                    {
                        int depthLinuxY = depthLinuxIndex / this.sendPointWidth;
                        int depthLinuxX = depthLinuxIndex - depthLinuxY * this.sendPointWidth;
                        int depthWindowsX = depthLinuxX + this.sendStartPointX;
                        int depthWindowsY = depthLinuxY + this.sendStartPointY;
                        int depthWindowsIndex = depthWindowsY * 512 + depthWindowsX;
                        boundsInWindows.Add(depthWindowsIndex);
                    }

                    int colorMinX = Convert.ToInt32(this.colorPoints[boundsInWindows[0]].X);
                    int colorMinY = Convert.ToInt32(this.colorPoints[boundsInWindows[1]].Y);
                    int colorMaxX = Convert.ToInt32(this.colorPoints[boundsInWindows[2]].X);
                    int colorMaxY = Convert.ToInt32(this.colorPoints[boundsInWindows[3]].Y);

                    // return when an invalid result is detected
                    if (colorMinX < 0 || colorMinY < 0 || colorMaxX >= 1920 || colorMaxY >= 1080
                        || colorMaxX < 0 || colorMaxY < 0 || colorMinX >= 1920 || colorMinY >= 1080)
                    {
                        return Task.FromResult(new Kinectrobot.BitStream { Status = false });
                    }

                    Kinectrobot.Bit color = new Kinectrobot.Bit
                    {
                        Name = "color",
                        X = colorMinX,
                        Y = colorMinY,
                        Width = colorMaxX - colorMinX,
                        Height = colorMaxY - colorMinY
                    };

                    result.Data.Add(color);
                }

                return Task.FromResult(result);
            }
            finally { this.pixelsLocker.ExitReadLock(); }
        }

        public override Task<DataStream> ReturnCognition(Request request, ServerCallContext context)
        {
            return Task.Run(() => HandleCognitionRequestAsync(request));
        }

        private async Task<DataStream> HandleCognitionRequestAsync(Request request)
        {
            // queue "send to cloud tasks" and create mask image for position detection
            float maskRatio = 0.3f; // parameter
            List<Task<Tuple<string, HttpResponseMessage>>> tasks = new List<Task<Tuple<string, HttpResponseMessage>>>();
            List<Kinectrobot.Bit> maskedImage = new List<Kinectrobot.Bit>();
            List<CameraSpacePoint> objectPositions = new List<CameraSpacePoint>();
            List<bool> valids = new List<bool>(); // far away positions are cut by valids

            foreach (var image in request.Data)
            {
                // initiate imageInSpace
                CameraSpacePoint initP = new CameraSpacePoint { X = 0.0f, Y = 0.0f, Z = 100.0f };
                objectPositions.Add(initP);

                // initiate valid
                valids.Add(true);

                // get partial image
                byte[] partial = new byte[Convert.ToInt32(image.Width) * Convert.ToInt32(image.Height) * 4];
                int atPixel = 0;
                for (int i = Convert.ToInt32(image.Y); i < Convert.ToInt32(image.Y) + Convert.ToInt32(image.Height); ++i)
                    for (int j = Convert.ToInt32(image.X) + Convert.ToInt32(image.Width) - 1; j >= Convert.ToInt32(image.X); --j) // flip image
                    {
                        int idx = (i * 1920 + j) * 4;
                        partial[atPixel++] = this.rawImage[idx++];
                        partial[atPixel++] = this.rawImage[idx++];
                        partial[atPixel++] = this.rawImage[idx++];
                        partial[atPixel++] = this.rawImage[idx++];
                    }

                // save to image
                string file = System.Environment.GetFolderPath(Environment.SpecialFolder.Personal) + image.Name + ".jpg";
                BitmapSource partialBitmapSource = BitmapSource.Create(Convert.ToInt32(image.Width), Convert.ToInt32(image.Height), 96, 96,
                    PixelFormats.Bgr32, null, partial, Convert.ToInt32(image.Width) * 4);
                System.Drawing.Bitmap partialBitmap;
                using (var ms = new MemoryStream())
                {
                    BitmapEncoder enc = new BmpBitmapEncoder();
                    enc.Frames.Add(BitmapFrame.Create(partialBitmapSource));
                    enc.Save(ms);
                    partialBitmap = new System.Drawing.Bitmap(ms);
                }
                using (Image resized = ResizeImage(partialBitmap, Convert.ToInt32(image.Width * 2), Convert.ToInt32(image.Height * 2)))
                {
                    SaveJpeg(file, resized, 100);
                }

                // add call cloud task
                tasks.Add(AnalyzeImage(request.Args, file));
                tasks.Add(OCR(request.Args, file));

                // create mask filter
                Kinectrobot.Bit bit = new Kinectrobot.Bit
                {
                    X = Convert.ToInt32(image.X + 0.5 * (1 - maskRatio) * image.Width),
                    Y = Convert.ToInt32(image.Y + 0.5 * (1 - maskRatio) * image.Height),
                    Width = maskRatio * image.Width,
                    Height = maskRatio * image.Height
                };
                maskedImage.Add(bit);
            }

            // get 3D position of image
            int pointIndex = 0;
            foreach (var point in this.cameraPointsColor)
            {
                // reject invalid points
                if (Double.IsInfinity(point.X) || Double.IsInfinity(point.Y) || Double.IsInfinity(point.Z)
                    || Double.IsNaN(point.X) || Double.IsNaN(point.Y) || Double.IsNaN(point.Z))
                {
                    ++pointIndex;
                    continue;
                }

                int maskIndex = 0;
                foreach (var image in maskedImage)
                {
                    int y = pointIndex / 1920;
                    int x = pointIndex - y * 1920;
                    // find point with closest depth && near image center
                    if (x >= image.X && y >= image.Y && x <= (image.X + image.Width) && y <= (image.Y + image.Height))
                        if (point.Z < objectPositions[maskIndex].Z)
                        {
                            CameraSpacePoint p = new CameraSpacePoint { X = point.X, Y = point.Y, Z = point.Z };
                            objectPositions[maskIndex] = p;
                        }
                    ++maskIndex;
                }

                ++pointIndex;
            }

            // check validity of 3D position
            int inSpaceIndex = 0;
            foreach (var image in objectPositions)
            {
                if (image.Z > 99.0f || image.Z < 0) valids[inSpaceIndex] = false;
                ++inSpaceIndex;
            }

            // get cloud result
            List<Tuple<string, float>> descriptions = new List<Tuple<string, float>>();
            List<List<Tuple<string, float>>> tags = new List<List<Tuple<string, float>>>();
            List<List<string>> texts = new List<List<string>>();
            int dataIndex = 0;

            await Task.WhenAny(Task.WhenAll(tasks), Task.Delay(120000));
            var completedTasks = tasks.Where(t => t.Status == TaskStatus.RanToCompletion).Select(t => t.Result).ToList();

            List<int> succeededTasks = new List<int>();
            int taskIndex = -1;
            foreach (var task in tasks)
            {
                ++taskIndex;
                if (task.Status == TaskStatus.RanToCompletion)
                    succeededTasks.Add(taskIndex);
            }

            int completedTaskIndex = -1;
            taskIndex = 0;
            foreach (var task in completedTasks)
            {
                ++completedTaskIndex;

                while (taskIndex != succeededTasks[completedTaskIndex])
                {
                    if (taskIndex % 2 == 0)
                    {
                        descriptions.Add(Tuple.Create("", 0.0f));
                        tags.Add(new List<Tuple<string, float>> { Tuple.Create("", 0.0f) });
                    }
                    else
                    {
                        texts.Add(new List<string> { "" });
                        ++dataIndex;
                    }
                    ++taskIndex;
                }

                ++taskIndex;

                if (!task.Item2.IsSuccessStatusCode)
                {
                    valids[dataIndex] = false;
                    if (task.Item1 == "analyze")
                    {
                        descriptions.Add(Tuple.Create("", 0.0f));
                        tags.Add(new List<Tuple<string, float>> { Tuple.Create("", 0.0f) });
                    }
                    else if (task.Item1 == "ocr")
                    {
                        texts.Add(new List<string> { "" });
                        ++dataIndex;
                    }
                    continue;
                }
                var cloudResult = await task.Item2.Content.ReadAsStringAsync();

                if (task.Item1 == "analyze")
                {
                    var analysisModel = JObject.Parse(cloudResult);

                    // add description
                    var descriptionModel = JObject.Parse(analysisModel["description"].ToString());
                    List<JToken> parsedCaptions = descriptionModel["captions"].ToObject<List<JToken>>();
                    foreach (var caption in parsedCaptions)
                    {
                        var captionModel = JObject.Parse(caption.ToString());
                        descriptions.Add(new Tuple<string, float>(captionModel["text"].ToObject<String>(), caption["confidence"].ToObject<float>()));
                        break; // only use first caption
                    }

                    // add tags
                    List<string> parsedTags = descriptionModel["tags"].ToObject<List<string>>();
                    tags.Add(new List<Tuple<string, float>>());
                    foreach (var tag in parsedTags)
                    {
                        tags[dataIndex].Add(new Tuple<string, float>(tag, 0.0f));
                    }
                    //List<JToken> parsedTags = analysisModel["tags"].ToObject<List<JToken>>();
                    //tags.Add(new List<Tuple<string, float>>());
                    //foreach (var tag in parsedTags)
                    //{
                    //    var tagModel = JObject.Parse(tag.ToString());
                    //    tags[dataIndex].Add(new Tuple<string, float>(tagModel["name"].ToObject<String>(), tagModel["confidence"].ToObject<float>()));
                    //}
                }
                else if (task.Item1 == "ocr")
                {
                    var ocrModel = JObject.Parse(cloudResult);

                    // add texts
                    List<JToken> parsedRegions = ocrModel["regions"].ToObject<List<JToken>>();
                    texts.Add(new List<string>());
                    foreach (var region in parsedRegions)
                    {
                        var regionModel = JObject.Parse(region.ToString());
                        List<JToken> parsedLines = regionModel["lines"].ToObject<List<JToken>>();
                        foreach (var line in parsedLines)
                        {
                            var lineModel = JObject.Parse(line.ToString());
                            List<JToken> parsedWords = lineModel["words"].ToObject<List<JToken>>();
                            foreach (var word in parsedWords)
                            {
                                var parsedWord = JObject.Parse(word.ToString());
                                texts[dataIndex].Add(parsedWord["text"].ToObject<String>());
                            }
                        }
                    }

                    ++dataIndex;
                }
            }

            while (taskIndex < tasks.Count)
            {
                if (taskIndex % 2 == 0)
                {
                    descriptions.Add(Tuple.Create("", 0.0f));
                    tags.Add(new List<Tuple<string, float>> { Tuple.Create("", 0.0f) });
                }
                else
                {
                    texts.Add(new List<string> { "" });
                    ++dataIndex;
                }
                ++taskIndex;
            }

            if (completedTasks.Count == 0)
            {
                return new Kinectrobot.DataStream { Status = false };
            }

            Kinectrobot.DataStream result = new Kinectrobot.DataStream { Status = true };

            for (int i = 0; i < request.Data.Count; ++i)
            {
                Kinectrobot.Data data = new Kinectrobot.Data
                {
                    Status = valids[i],
                    X = objectPositions[i].X,
                    Y = objectPositions[i].Y,
                    Z = objectPositions[i].Z
                };

                Kinectrobot.Tag caption = new Kinectrobot.Tag
                {
                    Tag_ = descriptions[i].Item1,
                    Confidence = descriptions[i].Item2
                };
                data.Captions.Add(caption);

                foreach (var tag in tags[i])
                {
                    Kinectrobot.Tag sendTag = new Kinectrobot.Tag
                    {
                        Tag_ = tag.Item1,
                        Confidence = tag.Item2
                    };
                    data.Tags.Add(sendTag);
                }

                foreach (var text in texts[i])
                {
                    data.Texts.Add(text);
                }

                result.Data.Add(data);
            }

            return result;
        }

        private async Task<Tuple<string, HttpResponseMessage>> AnalyzeImage(string key, string fileName)
        {
            //System.Threading.Thread.Sleep(80000); //sleep debug to simulate bad network

            var client = new HttpClient();
            var queryString = HttpUtility.ParseQueryString(string.Empty);

            // Request headers
            client.DefaultRequestHeaders.Add("Ocp-Apim-Subscription-Key", key);

            // Request parameters
            //queryString["visualFeatures"] = "Tags,Description";
            queryString["visualFeatures"] = "Description";
            var uri = "https://api.projectoxford.ai/vision/v1.0/analyze?" + queryString;

            HttpResponseMessage response;

            // Request body
            byte[] byteData = File.ReadAllBytes(fileName);

            using (var content = new ByteArrayContent(byteData))
            {
                content.Headers.ContentType = new MediaTypeHeaderValue("application/octet-stream");
                response = await client.PostAsync(uri, content);
            }

            return Tuple.Create("analyze", response);
        }

        private async Task<Tuple<string, HttpResponseMessage>> OCR(string key, string fileName)
        {
            var client = new HttpClient();
            var queryString = HttpUtility.ParseQueryString(string.Empty);

            // Request headers
            client.DefaultRequestHeaders.Add("Ocp-Apim-Subscription-Key", key);

            // Request parameters
            queryString["language"] = "en";
            queryString["detectOrientation "] = "true";
            var uri = "https://api.projectoxford.ai/vision/v1.0/ocr?" + queryString;

            HttpResponseMessage response;

            // Request body
            byte[] byteData = File.ReadAllBytes(fileName);

            using (var content = new ByteArrayContent(byteData))
            {
                content.Headers.ContentType = new MediaTypeHeaderValue("application/octet-stream");
                response = await client.PostAsync(uri, content);
            }

            return Tuple.Create("ocr", response);
        }

        private static System.Drawing.Bitmap ResizeImage(System.Drawing.Image image, int width, int height)
        {
            Bitmap result = new Bitmap(width, height);

            // use a graphics object to draw the resized image into the bitmap
            using (Graphics graphics = Graphics.FromImage(result))
            {
                // set the resize quality modes to high quality
                graphics.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                // draw the image into the target bitmap
                graphics.DrawImage(image, 0, 0, result.Width, result.Height);
            }

            //return the resulting bitmap
            return result;
        }

        private static void SaveJpeg(string path, Image image, int quality)
        {
            //ensure the quality is within the correct range
            if ((quality < 0) || (quality > 100))
            {
                //create the error message
                string error = string.Format("Expected quality between 0 and 100, with 100 highest quality. {0} was specified.", quality);
                //throw a helpful exception
                throw new ArgumentOutOfRangeException(error);
            }

            //create an encoder parameter for the image quality
            EncoderParameter qualityParam = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, quality);

            //get the jpeg codec
            string mimeType = "image/jpeg";
            //do a case insensitive search for the mime type
            string lookupKey = mimeType.ToLower();
            //the codec to return, default to null
            ImageCodecInfo jpegCodec = null;
            //if we have the encoder, get it to return
            foreach (ImageCodecInfo codec in ImageCodecInfo.GetImageEncoders())
                if (lookupKey == codec.MimeType.ToLower())
                {
                    jpegCodec = codec;
                    break;
                }

            //create a collection of all parameters that we will pass to the encoder
            EncoderParameters encoderParams = new EncoderParameters(1);
            //set the quality parameter for the codec
            encoderParams.Param[0] = qualityParam;
            //save the image using the codec and the parameters
            image.Save(path, jpegCodec, encoderParams);
        }

        public bool RecognitionOffTriggerOn()
        {
            bool abortFlag = false;

            locker.EnterReadLock();
            try { abortFlag = this.abortSpeech; }
            finally { locker.ExitReadLock(); }

            return abortFlag;
        }

        public void RecognitionOff()
        {
            locker.EnterWriteLock();
            try { this.abortSpeech = true; }
            finally { locker.ExitWriteLock(); }
        }

        public void RecognitionOn()
        {
            locker.EnterWriteLock();
            try { this.abortSpeech = false; }
            finally { locker.ExitWriteLock(); }
        }

        public override Task<Response> SendSpeech(Speech request, ServerCallContext context)
        {
            if (request.Command == "quit")
            {
                this.speechSynthesizer.SpeakAsyncCancelAll();
                return Task.FromResult(ReturnTrue());
            }

            this.speechSynthesizer.SpeakAsyncCancelAll();

            if (this.triggerDuringSpeech)
            {
                locker.EnterWriteLock();
                try { this.abortSpeech = true; }
                finally { locker.ExitWriteLock(); }
            }

            // example of speech
            // raw: "U--m... yeah?"
            // ssml: "<prosody rate=\"x-slow\">Um</prosody><break time=\"0.5s\"/><break time=\"0.5s\"/> yeah?"
            string raw_string = request.Speech_;
            // convert "--" syntax to x-slow
            while (true)
            {
                int getDashFromHead = raw_string.IndexOf("--");
                if (getDashFromHead < 0) break;
                // index should not change as erase and addition will happen on index behind
                int wordStart = raw_string.LastIndexOf(" ", getDashFromHead);
                int ssmlCheck = raw_string.LastIndexOf(">", getDashFromHead); // space could be inside ssml format
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf(".", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf(",", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf("?", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf("!", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = 0; // no spaces prior should mean head of sentence
                else wordStart += 1;
                // insert end first
                int wordEnd = raw_string.IndexOf(" ", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf(".", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf(",", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf("?", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf("!", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.Length - 1;
                raw_string = raw_string.Insert(wordEnd, "</prosody>");
                // erase dash dash
                raw_string = raw_string.Remove(getDashFromHead, 2);
                raw_string = raw_string.Insert(wordStart, "<prosody rate=\"x-slow\">");
            }
            // convert "..." syntax to break
            raw_string.Replace("...", "<break time=\"0.5s\">");
            Prompt speech = new Prompt("<?xml version=\"1.0\"?><speak version=\"1.0\" xml:lang=\"en-US\">" + raw_string + "</speak>", SynthesisTextFormat.Ssml);
            this.speechSynthesizer.SpeakAsync(speech);

            return Task.FromResult(ReturnTrue());
        }

        public void SpeechEnd_FrameArrived(object sender, SpeakCompletedEventArgs e)
        {
            if (this.triggerDuringSpeech)
            {
                locker.EnterWriteLock();
                try { this.abortSpeech = false; }
                finally { locker.ExitWriteLock(); }
            }
        }

        public override Task<Response> SetSTTBehavior(VoiceTriggers request, ServerCallContext context)
        {
            if (request.ManualTriggerOn)
            {
                this.abortSpeech = true;
                return Task.FromResult(ReturnTrue());
            }

            if (request.ManualTriggerOff)
            {
                this.abortSpeech = false;
                return Task.FromResult(ReturnTrue());
            }

            this.triggerAfterRecognition = request.AutoTriggerAfterRecognition;

            this.triggerDuringSpeech = request.AutoTriggerDuringSpeech;

            return Task.FromResult(ReturnTrue());
        }

        public override Task<Response> WebAgent(UrlInfo request, ServerCallContext context)
        {
            // if no search within webpage
            if (request.Key == "")
            {
                System.Diagnostics.Process.Start(request.Url);
                return Task.FromResult(ReturnTrue());
            }

            System.Net.WebClient wc = new System.Net.WebClient();
            string raw = wc.DownloadString(request.Url);
            List<string> substrings = new List<string> { };
            int getString = 0;
            string headString = request.Linkhead;
            string keyString = request.Key;
            while ((getString = raw.IndexOf(keyString, getString)) >= 0)
            {
                int findHref = raw.LastIndexOf("href=", getString) + 6;
                if (findHref < 0) continue;
                substrings.Add(headString + raw.Substring(findHref, getString - findHref) + keyString);
                getString += keyString.Length;
            }

            int order = -1;
            if (request.Style == "random") // return a random match
            {
                Random rand = new Random();
                System.Diagnostics.Process.Start(substrings[rand.Next(0, substrings.Count - 1)]);
            }
            else if (int.TryParse(request.Style, out order)) // return ith match
            {
                if (order < substrings.Count && order >= 0)
                    System.Diagnostics.Process.Start(substrings[order]);
            }
            
            wc.Dispose();

            return Task.FromResult(ReturnTrue());
        }

        private Response ReturnTrue()
        {
            return new Response { Status = true };
        }
    }
}
