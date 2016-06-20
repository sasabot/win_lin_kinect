//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using System.Windows;
//using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
//using System.Windows.Input;
//using System.Windows.Media;
//using System.Windows.Media.Imaging;
//using System.Windows.Navigation;
//using System.Windows.Shapes;

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

        // send-point parameters

        private int sendStartPointX = 95;

        private int sendStartPointY = 91;

        private int sendPointWidth = 320;

        private int sendPointHeight = 240;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.multiSourceFrameReader =
                    this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_FrameArrived;

            this.kinectSensor.Open();

            InitializeComponent();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            this.channel = new Channel("192.168.101.1:50052", Credentials.Insecure);
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
            // the code crashes with 30fps, set parameter according to computer specifications
            if (this.skippedFrame < 30)
            {
                ++skippedFrame;
                return;
            }

            skippedFrame = 0;

            MultiSourceFrame frame = e.FrameReference.AcquireFrame();

            if (frame == null)
                return;
            
            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null | depthFrame == null)
                return;

            // set point cloud to send (iterate through depth map)
            SendPoints(depthFrame, colorFrame).Wait();

            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        public async Task SendPoints(DepthFrame depthFrame, ColorFrame colorFrame)
        {
            // get depth map from depthFrame
            var depthDesc = depthFrame.FrameDescription;
            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            // get color pixels from colorFrame
            var colorDesc = colorFrame.FrameDescription;
            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * 4];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            //CameraSpacePoint[] cameraPoints = new CameraSpacePoint[colorDesc.Width * colorDesc.Height];
            //this.coordinateMapper.MapColorFrameToCameraSpace(depthData, cameraPoints);

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
            List<Kinectrgbd.Point> points = new List<Kinectrgbd.Point>();
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
                points.Add(p);               
            }

            try
            {
                using (var call = client.SendPoints())
                {
                    foreach(var point in points)
                    {
                        await call.RequestStream.WriteAsync(point);
                    }
                    await call.RequestStream.CompleteAsync();

                    Kinectrgbd.Response res = await call.ResponseAsync;
                }
            }
            catch (RpcException e)
            {
                throw;
            }
        } // SendPoints

    }
}
