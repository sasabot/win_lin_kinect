using System;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.System;
using Windows.UI.ViewManagement;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;

namespace WindowsKinectLaunch
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage() {
            this.InitializeComponent();

            ApplicationView.PreferredLaunchViewSize = new Size(200, 100);
            ApplicationView.PreferredLaunchWindowingMode = ApplicationViewWindowingMode.PreferredLaunchViewSize;

            var task1 = Task.Run(async () => {
                var rgbdUri = new Uri("kinectrgbdinteraction:");
                await Launcher.LaunchUriAsync(rgbdUri);
            });

            var task2 = Task.Run(async () => {
                var rgbdUri = new Uri("kinectwindowsinteraction:");
                await Launcher.LaunchUriAsync(rgbdUri);
            });

            var task3 = Task.Run(async () => {
                var rgbdUri = new Uri("kinectmicrophoneinteraction:");
                await Launcher.LaunchUriAsync(rgbdUri);
            });

            task1.Wait();
            task2.Wait();
            task3.Wait();

            Application.Current.Exit();
        }
    }
}
