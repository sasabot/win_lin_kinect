#include "pch.h"
#include "Class1.h"

using namespace HaarRuntimeComponent;
using namespace Platform;

Class1::Class1(Platform::String^ _xml) : headDetector() {
	auto fn = std::wstring(_xml->Data());
	auto xml = std::string(fn.begin(), fn.end());

	cv::FileStorage fs(xml, cv::FileStorage::READ | cv::FileStorage::MEMORY);
	headDetector.read(fs.getFirstTopLevelNode());
}

Windows::Foundation::Collections::IVector<uint8>^ Class1::DetectMultiScale(const Platform::Array<uint8>^ image, int inwidth, int inheight) {
	cv::Mat img(inheight, inwidth, CV_8U);
	
	int i = 0;
	int j = 0;
	for (int k = 0; k < image->Length; ++k) {
		img.at<char>(i, j++) = image[k];
		if (j >= img.cols) {
			j = 0;
			++i;
		}
	}

	std::vector<cv::Rect> heads;
	headDetector.detectMultiScale(img, heads);

	auto res = ref new Platform::Collections::Vector<uint8>();
	res->Append(heads.size());
	for (auto it = heads.begin(); it != heads.end(); ++it) {
		res->Append(static_cast<uint>(it->x));
		res->Append(static_cast<uint>(it->y));
		res->Append(static_cast<uint>(it->width));
		res->Append(static_cast<uint>(it->height));
	}

	return res;
}
