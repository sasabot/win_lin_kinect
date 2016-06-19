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
        List<Kinectrgbd.Point> saved_points = new List<Kinectrgbd.Point>();
        bool respondSet = true;

        public KinectRgbdImpl()
        {

        }

        public async Task GetPoints(Kinectrgbd.Request request,
            Grpc.Core.IServerStreamWriter<Kinectrgbd.Point> responseStream,
            Grpc.Core.ServerCallContext context)
        {
            respondSet = false;
            int point_count = 0;
            foreach (Kinectrgbd.Point point in this.saved_points)
            {
                await responseStream.WriteAsync(point);
                ++point_count;
                if (point_count > 20000) break;
            }
            this.saved_points.Clear();
            respondSet = true;
        }

        public void ClearPoints()
        {
            this.points.Clear();
        }

        public void SetPoint(Kinectrgbd.Point point)
        {
            this.points.Add(point);
        }

        public void SavePoints()
        {
            if (!respondSet) return;

            foreach (Kinectrgbd.Point point in this.points)
            {
                this.saved_points.Add(point);
            }
        }
    }
}
