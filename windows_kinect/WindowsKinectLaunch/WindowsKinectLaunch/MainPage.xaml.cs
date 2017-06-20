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
                var uri = new Uri("kinectrgbdinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            var task2 = Task.Run(async () => {
                var uri = new Uri("kinectwindowsinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            var task3 = Task.Run(async () => {
                var uri = new Uri("kinectmicrophoneinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            var task4 = Task.Run(async () => {
                var uri = new Uri("kinectfaceinteraction:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            var task5 = Task.Run(async () => {
                var uri = new Uri("kinectimagestreamer:");
                var status = await Launcher.QueryUriSupportAsync(uri, LaunchQuerySupportType.Uri);
                if (status == LaunchQuerySupportStatus.Available)
                    await Launcher.LaunchUriAsync(uri);
            });

            task1.Wait();
            task2.Wait();
            task3.Wait();
            task4.Wait();
            task5.Wait();

            Application.Current.Exit();
        }
    }
}
