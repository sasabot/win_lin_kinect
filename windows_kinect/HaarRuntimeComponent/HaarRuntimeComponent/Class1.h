#pragma once

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

namespace HaarRuntimeComponent
{
    public ref class Class1 sealed
    {
    public:
        Class1(Platform::String^ _xml);

		Windows::Foundation::Collections::IVector<uint8>^ DetectMultiScale(const Platform::Array<uint8>^ image, int inwidth, int inheight);

	private:
		cv::CascadeClassifier headDetector;
    };
}
