using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Grpc.Core.Utils;

namespace Kinectrgbd
{
    public class KinectRgbdImpl : Kinectrgbd.KinectRgbd.IKinectRgbd
    {
        List<Kinectrgbd.Point> points = new List<Kinectrgbd.Point>();

        public KinectRgbdImpl()
        {

        }

        public async Task GetPoints(Kinectrgbd.Request request,
            Grpc.Core.IServerStreamWriter<Kinectrgbd.Point> responseStream,
            Grpc.Core.ServerCallContext context)
        {
            foreach (Kinectrgbd.Point point in points)
            {
                await responseStream.WriteAsync(point);
            }
        }

        public void ClearPoints()
        {
            this.points.Clear();
        }

        public void SetPoint(Kinectrgbd.Point point)
        {
            this.points.Add(point);
        }
    }
}
