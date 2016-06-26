using Grpc.Core;

namespace KinectSimpleRgbdServer
{
    using System;
    using System.Collections.Concurrent;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
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
            this.skippedFrame = KinectSimpleRgbdServer.MainWindow.frameUnit * this.fps;
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
            this.skippedFrame = KinectSimpleRgbdServer.MainWindow.frameUnit * this.fps;
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

        private void SendDepthCloud(MultiSourceFrame frame)
        {
            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null | depthFrame == null)
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
                if (img_x >= 0 && img_y >= 0 && img_x < 1960 && img_y < 1080)
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
            if (colorFrame == null | depthFrame == null)
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
