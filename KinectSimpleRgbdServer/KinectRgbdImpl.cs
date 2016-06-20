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
        private List<Kinectrgbd.Point> saved_points = new List<Kinectrgbd.Point>();

        private bool points_lock = false;

        public KinectRgbdImpl()
        {

        }

        public async Task GetPoints(Kinectrgbd.Request request,
            Grpc.Core.IServerStreamWriter<Kinectrgbd.Point> responseStream,
            Grpc.Core.ServerCallContext context)
        {
            if (this.points_lock) return;
            this.points_lock = true;
            foreach (Kinectrgbd.Point point in this.saved_points)
            {
                await responseStream.WriteAsync(point);
            }
            this.saved_points.Clear();
            this.points_lock = false;
        }

        public void SetPoint(Kinectrgbd.Point point)
        {
            this.saved_points.Add(point);
        }

        public bool SetLockState()
        {
            if (this.points_lock == false)
            {
                this.points_lock = true;
                return true;
            }

            return false;
        }

        public void FreeLockState()
        {
            this.points_lock = false;
        }
    }
}
