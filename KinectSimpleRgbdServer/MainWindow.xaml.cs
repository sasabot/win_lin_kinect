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

        private Kinectrgbd.KinectRgbdImpl streamer =
            new Kinectrgbd.KinectRgbdImpl();

        private Grpc.Core.Server myserver;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.multiSourceFrameReader =
                    this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);

            this.kinectSensor.Open();

            InitializeComponent();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            this.myserver = new Grpc.Core.Server
            {
                Services = { Kinectrgbd.KinectRgbd.BindService(this.streamer) },
                Ports = { new ServerPort("192.168.101.190", 50052, Grpc.Core.ServerCredentials.Insecure) }
            };
            this.myserver.Start();
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            this.myserver.ShutdownAsync().Wait();
        }

        private void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame frame = e.FrameReference.AcquireFrame();

            if (frame == null)
                return;
            
            ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null | depthFrame == null)
                return;

            var depthDesc = depthFrame.FrameDescription;

            ushort[] depthData = new ushort[depthDesc.Width * depthDesc.Height];
            depthFrame.CopyFrameDataToArray(depthData);

            var colorDesc = colorFrame.FrameDescription;

            CameraSpacePoint[] cameraPoints = new CameraSpacePoint[colorDesc.Width * colorDesc.Height];
            this.coordinateMapper.MapColorFrameToCameraSpace(depthData, cameraPoints);

            byte[] pixels = new byte[colorDesc.Width * colorDesc.Height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];
            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

            this.streamer.ClearPoints();
            int colorIndex = 0;
            foreach (CameraSpacePoint point in cameraPoints)
            {
                Kinectrgbd.Point p = new Kinectrgbd.Point
                {
                    Color = new Kinectrgbd.Color { B = pixels[colorIndex++], G = pixels[colorIndex++], R = pixels[colorIndex++] },
                    Position = new Kinectrgbd.Position {  X = point.X, Y = point.Y, D = point.Z }
                };
                ++colorIndex;
                this.streamer.SetPoint(p);
            }
        }
    }
}
