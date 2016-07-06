using Grpc.Core;

namespace KinectSimpleRgbdServer
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Net.Http;
    using System.Net.Http.Headers;
    using System.Web;
    using System.Drawing;
    using System.Drawing.Imaging;
    using System.IO;
    using System.Threading.Tasks;
    using Newtonsoft.Json.Linq;
    using System.Linq;
    

    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor = null;

        private CoordinateMapper coordinateMapper = null;

        private MultiSourceFrameReader multiSourceFrameReader = null;

        private Channel channel = null;

        private Kinectrgbd.KinectRgbd.KinectRgbdClient client;

        private int skippedFrame = 0;

        private int fps = 1; // note this sets seconds per frame

        private const int frameUnit = 30;

        private List<CameraSpacePoint> imageSpaceValues;

        // parameters from request

        private int mode = 0;

        private int sendStartPointX = 95;

        private int sendStartPointY = 91;

        private int sendPointWidth = 320;

        private int sendPointHeight = 240;

        private bool onceFlag = false;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.multiSourceFrameReader =
                    this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_FrameArrived;

            this.kinectSensor.Open();

            this.imageSpaceValues = new List<CameraSpacePoint>();

            InitializeComponent();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            this.channel = new Channel("192.168.101.192:50052", Credentials.Insecure);
            this.client = Kinectrgbd.KinectRgbd.NewClient(this.channel);
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            this.channel.ShutdownAsync().Wait();
        }

        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // the code might crash with 30fps, set parameter according to computer specifications
            if (this.skippedFrame < KinectSimpleRgbdServer.MainWindow.frameUnit * this.fps)
            {
                ++skippedFrame;
                return;
            }

            skippedFrame = 0;

            if (this.mode == 0) // check if there is a request
            {
                Kinectrgbd.Header header = new Kinectrgbd.Header { Data = true };
                Kinectrgbd.Request request = this.client.CheckRequest(header);
                if (request.Mode == 0) return;
                else if (request.Mode == 1) PrepareRgbdMode(request);
                else if (request.Mode == 2) PrepareImageMode(request);
                else if (request.Mode == 3) HandlePositionRequest(request); // send physical position of pixel from saved info
                else if (request.Mode == 4) // send bounding box in depth / pixel coords from space coords
                {
                    HandleBoundingRequest(request, e.FrameReference.AcquireFrame());
                    PrepareRequestMode();
                    return;
                }
                else if (request.Mode == 5) // call Microsoft cognitive services from partial image streams
                {
                    HandleCognitiveRequest(request, e.FrameReference.AcquireFrame());
                    PrepareRequestMode();
                    return;
                }
            }

            MultiSourceFrame frame = e.FrameReference.AcquireFrame();

            if (frame == null)
                return;
            
            if (this.mode == 1) // send rgbd
            {
                SendDepthCloud(frame);
            }
            else if (this.mode == 2) // send image and save frame info
            {
                SendImage(frame);
            }

        }

        private void PrepareRequestMode()
        {
            this.mode = 0;
            this.fps = 1;
        }

        private void PrepareRgbdMode(Kinectrgbd.Request request)
        {
            this.sendStartPointX = Convert.ToInt32(request.Data[0].X);
            this.sendStartPointY = Convert.ToInt32(request.Data[0].Y);
            this.sendPointWidth = Convert.ToInt32(request.Data[0].Width);
            this.sendPointHeight = Convert.ToInt32(request.Data[0].Height);
            this.onceFlag = request.Once;
            this.mode = 1;
            this.fps = 2;
            //this.skippedFrame = KinectSimpleRgbdServer.MainWindow.frameUnit * this.fps;
        }

        private void PrepareImageMode(Kinectrgbd.Request request)
        {
            this.sendStartPointX = Convert.ToInt32(request.Data[0].X);
            this.sendStartPointY = Convert.ToInt32(request.Data[0].Y);
            this.sendPointWidth = Convert.ToInt32(request.Data[0].Width);
            this.sendPointHeight = Convert.ToInt32(request.Data[0].Height);
            this.onceFlag = request.Once;
            this.mode = 2;
            this.fps = 3;
            //this.skippedFrame = KinectSimpleRgbdServer.MainWindow.frameUnit * this.fps;
        }

        private void HandlePositionRequest(Kinectrgbd.Request request)
        {
            Kinectrgbd.DataStream result = new Kinectrgbd.DataStream { Status = true };

            for (int i = 0; i < request.Data.Count; ++i)
            {
                int pixelIndex = Convert.ToInt32(request.Data[i].Y) * Convert.ToInt32(request.Data[i].Width)
                    + Convert.ToInt32(request.Data[i].X);
                if (pixelIndex >= this.imageSpaceValues.Count)
                {
                    result.Status = false;
                    continue;
                }
                Kinectrgbd.Data data = new Kinectrgbd.Data
                { 
                    X = this.imageSpaceValues[pixelIndex].X,
                    Y = this.imageSpaceValues[pixelIndex].Y,
                    Z = this.imageSpaceValues[pixelIndex].Z
                };
                result.Data.Add(data);
            }
            
            Kinectrgbd.Response response = this.client.ReturnPositionsFromPixels(result);
        }

        private void HandleBoundingRequest(Kinectrgbd.Request request, MultiSourceFrame frame)
        {
            Kinectrgbd.BitStream result = new Kinectrgbd.BitStream { Status = false };

            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null || depthFrame == null)
            {
                Kinectrgbd.Response badresponse = this.client.ReturnPixelBoundsFromSpaceBounds(result);
                return;
            }

            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            // get physical xyz position for each point on depth map
            CameraSpacePoint[] cameraPoints = new CameraSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToCameraSpace(depthData, cameraPoints);

            // get corresponding color pixel for each point on depth map
            ColorSpacePoint[] colorPoints = new ColorSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToColorSpace(depthData, colorPoints);

            // draw camera image
            BitmapSource bitmapSource = BitmapSource.Create(colorDesc.Width, colorDesc.Height, 96, 96,
                PixelFormats.Bgr32, null, pixels, colorDesc.Width * 4);
            this.canvas.Background = new ImageBrush(bitmapSource);

            float referenceXMin = request.Data[0].X;
            float referenceYMin = request.Data[0].Y;
            float minimumMinDist = 100;
            int depthPixelMin = -1;
            float referenceXMax = request.Data[0].X + request.Data[0].Width;
            float referenceYMax = request.Data[0].Y + request.Data[0].Height;
            float minimumMaxDist = 100;
            int depthPixelMax = -1;
            int pointIndex = 0;
            foreach (var point in cameraPoints)
            {
                // reject invalid points
                if (Double.IsInfinity(point.X) || Double.IsInfinity(point.Y) || Double.IsInfinity(point.Z)
                    || Double.IsNaN(point.X) || Double.IsNaN(point.Y) || Double.IsNaN(point.Z))
                {
                    ++pointIndex;
                    continue;
                }

                // reject points without color correspondent
                int img_y = -1;
                int img_x = -1;
                if (!Double.IsInfinity(colorPoints[pointIndex].X) && !Double.IsInfinity(colorPoints[pointIndex].Y)
                    && !Double.IsNaN(colorPoints[pointIndex].X) && !Double.IsNaN(colorPoints[pointIndex].Y))
                {
                    img_y = Convert.ToInt32(colorPoints[pointIndex].Y);
                    img_x = Convert.ToInt32(colorPoints[pointIndex].X);
                }
                if (img_x < 0 || img_y < 0 || img_x >= colorDesc.Width || img_y >= colorDesc.Height)
                {
                    ++pointIndex;
                    continue;
                }

                float distanceMin =
                    (point.X - referenceXMin) * (point.X - referenceXMin) + (point.Y - referenceYMin) * (point.Y - referenceYMin);
                if (distanceMin < minimumMinDist)
                {
                    minimumMinDist = distanceMin;
                    depthPixelMin = pointIndex;
                }
                float distanceMax =
                    (point.X - referenceXMax) * (point.X - referenceXMax) + (point.Y - referenceYMax) * (point.Y - referenceYMax);
                if (distanceMax < minimumMaxDist)
                {
                    minimumMaxDist = distanceMax;
                    depthPixelMax = pointIndex;
                }

                ++pointIndex;
            }

            if (depthPixelMin < 0 || depthPixelMax < 0)
            {
                Kinectrgbd.Response badresponse = this.client.ReturnPixelBoundsFromSpaceBounds(result);
                colorFrame.Dispose();
                depthFrame.Dispose();
                return;
            }

            int depthMinY = depthPixelMin / depthDesc.Width;
            int depthMinX = depthPixelMin - depthMinY * depthDesc.Width;
            int depthMaxY = depthPixelMax / depthDesc.Width;
            int depthMaxX = depthPixelMax - depthMaxY * depthDesc.Width;
            Kinectrgbd.Bit depth = new Kinectrgbd.Bit
            {
                Name = "depth",
                X = depthMinX,
                Y = depthMaxY,
                Width = depthMaxX - depthMinX,
                Height = depthMinY - depthMaxY
            };
            int colorMinX = Convert.ToInt32(colorPoints[depthPixelMin].X);
            int colorMinY = Convert.ToInt32(colorPoints[depthPixelMin].Y);
            int colorMaxX = Convert.ToInt32(colorPoints[depthPixelMax].X);
            int colorMaxY = Convert.ToInt32(colorPoints[depthPixelMax].Y);
            Kinectrgbd.Bit color = new Kinectrgbd.Bit
            { 
                Name = "color",
                X = colorMinX,
                Y = colorMaxY,
                Width = colorMaxX - colorMinX,
                Height = colorMinY - colorMaxY
            };

            result.Status = true;
            result.Data.Add(depth);
            result.Data.Add(color);
            Kinectrgbd.Response response = this.client.ReturnPixelBoundsFromSpaceBounds(result);

            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        private async void HandleCognitiveRequest(Kinectrgbd.Request request, MultiSourceFrame frame)
        {
            Kinectrgbd.DataStream result = new Kinectrgbd.DataStream { Status = false };

            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();

            if (colorFrame == null || depthFrame == null)
            {
                Kinectrgbd.Response badresponse = this.client.ReturnCognition(result);
                return;
            }

            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            // get physical xyz position for each point on depth map
            CameraSpacePoint[] cameraPoints = new CameraSpacePoint[colorDesc.Width * colorDesc.Height];
            this.coordinateMapper.MapColorFrameToCameraSpace(depthData, cameraPoints);

            // draw camera image
            BitmapSource bitmapSource = BitmapSource.Create(colorDesc.Width, colorDesc.Height, 96, 96,
                PixelFormats.Bgr32, null, pixels, colorDesc.Width * 4);
            this.canvas.Background = new ImageBrush(bitmapSource);

            colorFrame.Dispose();
            depthFrame.Dispose();

            // cognition task must run on different thread or will laten kinect frames
            Task.Run(() => HandleCognitionRequestAsync(request, pixels, cameraPoints, depthDesc, colorDesc, result));
        }

        private async Task HandleCognitionRequestAsync(Kinectrgbd.Request request, byte[] pixels, CameraSpacePoint[] cameraPoints,
            FrameDescription depthDesc, FrameDescription colorDesc, Kinectrgbd.DataStream result)
        {

            // setup async cloud tasks and create mask image for position detection
            float maskRatio = 0.3f; // parameter
            List<CameraSpacePoint> imageInSpace = new List<CameraSpacePoint>();
            List<bool> valids = new List<bool>();
            List<Task<Tuple<string, HttpResponseMessage>>> tasks = new List<Task<Tuple<string, HttpResponseMessage>>>();
            List<Kinectrgbd.Bit> maskedImage = new List<Kinectrgbd.Bit>();
            foreach (var image in request.Data)
            {
                // initiate imageInSpace
                CameraSpacePoint initP = new CameraSpacePoint { X = 0.0f, Y = 0.0f, Z = 100.0f };
                imageInSpace.Add(initP);

                // initiate valid
                valids.Add(true);

                // get partial image
                byte[] partial = new byte[Convert.ToInt32(image.Width) * Convert.ToInt32(image.Height) * 4];
                int atPixel = 0;
                for (int i = Convert.ToInt32(image.Y); i < Convert.ToInt32(image.Y) + Convert.ToInt32(image.Height); ++i)
                    for (int j = Convert.ToInt32(image.X) + Convert.ToInt32(image.Width) - 1; j >= Convert.ToInt32(image.X); --j) // flip image
                    {
                        int idx = (i * colorDesc.Width + j) * 4;
                        partial[atPixel++] = pixels[idx++];
                        partial[atPixel++] = pixels[idx++];
                        partial[atPixel++] = pixels[idx++];
                        partial[atPixel++] = pixels[idx++];
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
                Kinectrgbd.Bit bit = new Kinectrgbd.Bit
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
            foreach (var point in cameraPoints)
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
                    int y = pointIndex / colorDesc.Width;
                    int x = pointIndex - y * colorDesc.Width;
                    // find point with closest depth && near image center
                    if (x >= image.X && y >= image.Y && x <= (image.X + image.Width) && y <= (image.Y + image.Height))
                        if (point.Z < imageInSpace[maskIndex].Z)
                        {
                            CameraSpacePoint p = new CameraSpacePoint { X = point.X, Y = point.Y, Z = point.Z };
                            imageInSpace[maskIndex] = p;
                        }
                    ++maskIndex;
                }

                ++pointIndex;
            }

            // check validity of 3D position
            int inSpaceIndex = 0;
            foreach (var image in imageInSpace)
            {
                if (image.Z > 99.0f || image.Z < 0) valids[inSpaceIndex] = false;
                ++inSpaceIndex;
            }

#if true
            // get cloud result
            List<Tuple<string, float>> descriptions = new List<Tuple<string, float>>();
            List<List<Tuple<string, float>>> tags = new List<List<Tuple<string, float>>>();
            List<List<string>> texts = new List<List<string>>();
            int dataIndex = 0;

            //await Task.WhenAny(Task.WhenAll(tasks), Task.Delay(3000));
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

            //foreach (var task in await Task.WhenAll(tasks))
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
                Kinectrgbd.Response badresponse = this.client.ReturnCognition(result);
                return;
            }
#else
            for (int i = 0; i < request.Data.Count; ++i)
            {
                Kinectrgbd.Data data = new Kinectrgbd.Data
                {
                    Status = valids[i],
                    X = imageInSpace[i].X,
                    Y = imageInSpace[i].Y,
                    Z = imageInSpace[i].Z
                };

                result.Data.Add(data);
            }
#endif

#if true
            for (int i = 0; i < request.Data.Count; ++i)
            {
                Kinectrgbd.Data data = new Kinectrgbd.Data
                {
                    Status = valids[i],
                    X = imageInSpace[i].X,
                    Y = imageInSpace[i].Y,
                    Z = imageInSpace[i].Z
                };

                Kinectrgbd.Tag caption = new Kinectrgbd.Tag
                {
                    Tag_ = descriptions[i].Item1,
                    Confidence = descriptions[i].Item2
                };
                data.Captions.Add(caption);

                foreach (var tag in tags[i])
                {
                    Kinectrgbd.Tag sendTag = new Kinectrgbd.Tag
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
#endif

            result.Status = true;
            Kinectrgbd.Response response = this.client.ReturnCognition(result);

            //colorFrame.Dispose();
            //depthFrame.Dispose();
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
            // set the resolutions the same to avoid cropping due to resolution differences
            //result.SetResolution(image.HorizontalResolution, image.VerticalResolution);

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

        private void SendDepthCloud(MultiSourceFrame frame)
        {
            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null || depthFrame == null)
                return;

            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            // get physical xyz position for each point on depth map
            CameraSpacePoint[] cameraPoints = new CameraSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToCameraSpace(depthData, cameraPoints);

            // get corresponding color pixel for each point on depth map
            ColorSpacePoint[] colorPoints = new ColorSpacePoint[depthDesc.Width * depthDesc.Height];
            this.coordinateMapper.MapDepthFrameToColorSpace(depthData, colorPoints);

            // draw camera image
            BitmapSource bitmapSource = BitmapSource.Create(colorDesc.Width, colorDesc.Height, 96, 96,
                PixelFormats.Bgr32, null, pixels, colorDesc.Width * 4);
            this.canvas.Background = new ImageBrush(bitmapSource);

            int pointIndex = 0;
            int sendEndPointX = this.sendStartPointX + this.sendPointWidth;
            int sendEndPointY = this.sendStartPointY + this.sendPointHeight;
            Kinectrgbd.Points points = new Kinectrgbd.Points { };
            foreach (CameraSpacePoint point in cameraPoints)
            {
                //if (Double.IsInfinity(point.X) || Double.IsInfinity(point.Y) || Double.IsInfinity(point.Z)
                //    || Double.IsNaN(point.X) || Double.IsNaN(point.Y) || Double.IsNaN(point.Z))
                //{
                //    ++pointIndex;
                //    continue;
                //}

                // get points only in region
                int pointYonDepth = pointIndex / depthDesc.Width;
                int pointXonDepth = pointIndex - depthDesc.Width * pointYonDepth;
                if (pointXonDepth < this.sendStartPointX || pointYonDepth < this.sendStartPointY
                    || pointXonDepth >= sendEndPointX || pointYonDepth >= sendEndPointY)
                {
                    ++pointIndex;
                    continue;
                }

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
                //else
                //{
                //    ++pointIndex;
                //    continue;
                //}

                // add point to stream
                Kinectrgbd.Point p = new Kinectrgbd.Point
                {
                    Color = color,
                    X = point.X,
                    Y = point.Y,
                    Z = point.Z
                };
                ++pointIndex;
                points.Data.Add(p);
            }

            // set point cloud to send
            Kinectrgbd.Response response = this.client.SendPoints(points);

            // stop streaming if once or stop is requested
            if (this.onceFlag || response.Finish) PrepareRequestMode();

            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        private void SendImage(MultiSourceFrame frame)
        {
            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null || depthFrame == null)
                return;

            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            // get physical xyz position for each pixel on color map
            CameraSpacePoint[] cameraPoints = new CameraSpacePoint[colorDesc.Width * colorDesc.Height];
            this.coordinateMapper.MapColorFrameToCameraSpace(depthData, cameraPoints);

            // draw camera image
            BitmapSource bitmapSource = BitmapSource.Create(colorDesc.Width, colorDesc.Height, 96, 96,
                PixelFormats.Bgr32, null, pixels, colorDesc.Width * 4);
            this.canvas.Background = new ImageBrush(bitmapSource);

            int pixelIndex = 0;
            int sendEndPointX = this.sendStartPointX + this.sendPointWidth;
            int sendEndPointY = this.sendStartPointY + this.sendPointHeight;
            this.imageSpaceValues.Clear();
            Kinectrgbd.Pixels result = new Kinectrgbd.Pixels {};
            foreach (CameraSpacePoint spaceValue in cameraPoints)
            {
                // get pixels only in region
                int pixelYonImage = pixelIndex / colorDesc.Width;
                int pixelXonImage = pixelIndex - colorDesc.Width * pixelYonImage;
                if (pixelXonImage < this.sendStartPointX || pixelYonImage< this.sendStartPointY
                    || pixelXonImage >= sendEndPointX || pixelYonImage >= sendEndPointY)
                {
                    ++pixelIndex;
                    continue;
                }

                // get color
                int pixel = 4 * pixelIndex; // bgra, so skip by 4
                int color = ((pixels[pixel++] << 16) & 0xfffffff) + ((pixels[pixel++] << 8) & 0xfffffff) + pixels[pixel++];

                // get space position
                this.imageSpaceValues.Add(spaceValue);

                // add pixel to stream
                ++pixelIndex;
                result.Color.Add(color);
            }

            // set image pixels to send (iterate through color map)
            Kinectrgbd.Response response = this.client.SendImage(result);

            // stop streaming if once or stop is requested
            if (this.onceFlag || response.Finish) PrepareRequestMode();

            colorFrame.Dispose();
            depthFrame.Dispose();
        }


    }
}
