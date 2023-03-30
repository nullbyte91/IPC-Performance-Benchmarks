#include <iostream>
#include <omniORB4/CORBA.h>
#include <omniORB4/Naming.hh>
#include "ImageTransfer.hh"
#include <opencv2/opencv.hpp>

using namespace ImageTransfer;

int main(int argc, char **argv)
{
    try
    {
        CORBA::ORB_var orb = CORBA::ORB_init(argc, argv);

        CORBA::Object_var obj = orb->resolve_initial_references("NameService");
        CosNaming::NamingContext_var rootContext = CosNaming::NamingContext::_narrow(obj);

        if (CORBA::is_nil(rootContext))
        {
            std::cerr << "Failed to narrow NameService" << std::endl;
            return 1;
        }

        CosNaming::Name contextName;
        contextName.length(1);
        contextName[0].id = (const char *)"ImageService";
        contextName[0].kind = (const char *)"object";

        CORBA::Object_var imageServiceObj = rootContext->resolve(contextName);
        ImageService_var imageService = ImageService::_narrow(imageServiceObj);

        if (CORBA::is_nil(imageService))
        {
            std::cerr << "Failed to narrow ImageService" << std::endl;
            return 1;
        }

        cv::Mat img = cv::imread("./other-git/opencv/doc/tutorials/imgproc/histograms/histogram_equalization/images/Histogram_Equalization_Theory_2.jpg", cv::IMREAD_COLOR);
        if (img.empty())
        {
            std::cerr << "Failed to read image" << std::endl;
            return 1;
        }

        std::string image_data(reinterpret_cast<char *>(img.data), img.total() * img.elemSize());

        CORBA::Boolean result = imageService->send_image(image_data.c_str(), img.cols, img.rows, img.type());

        if (result)
        {
            std::cout << "Image sent successfully" << std::endl;
        }
        else
        {
            std::cerr << "Failed to send image" << std::endl;
            return 1;
        }

        orb->destroy();
    }
    catch (const CORBA::Exception &ex)
    {
        std::cerr << "Uncaught CORBA exception: " << ex._name() << std::endl;
        return 1;
    }

    return 0;
}
