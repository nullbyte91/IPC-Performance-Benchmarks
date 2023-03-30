#include <iostream>
#include <omniORB4/CORBA.h>
#include <omniORB4/Naming.hh>
#include "ImageTransfer.hh"
#include <opencv2/opencv.hpp>

using namespace ImageTransfer;

class ImageServiceImpl : public POA_ImageTransfer::ImageService
{
public:
    CORBA::Boolean send_image(const char *image_data, CORBA::Long width, CORBA::Long height, CORBA::Long type)
    {
        cv::Mat img(height, width, type, (void *)image_data);

        // Process the image, e.g., display or save it
        std::cout << "Received image: " << img.rows << " * " << img.cols << std::endl;

        return true;
    }
};

int main(int argc, char **argv)
{
    try
    {
        const char *orb_args[] = {
        argv[0],
        "-ORBgiopMaxMsgSize", "4194304"
        };
        int orb_argc = sizeof(orb_args) / sizeof(char *);

        CORBA::ORB_var orb = CORBA::ORB_init(orb_argc, const_cast<char **>(orb_args));

        PortableServer::POA_var poa = PortableServer::POA::_narrow(orb->resolve_initial_references("RootPOA"));

        ImageServiceImpl *image_service = new ImageServiceImpl();

        PortableServer::ObjectId_var image_service_oid = poa->activate_object(image_service);

        CORBA::Object_var obj = poa->id_to_reference(image_service_oid.in());

        if (CORBA::is_nil(obj.in()))
        {
            std::cerr << "Can't get reference to implemented object." << std::endl;
            return 1;
        }

        CORBA::String_var sior(orb->object_to_string(obj.in()));
        std::cout << "Object reference: " << (char *)sior << std::endl;

        // Bind to the naming service
        CosNaming::NamingContext_var rootContext;
        try
        {
            CORBA::Object_var obj = orb->resolve_initial_references("NameService");
            rootContext = CosNaming::NamingContext::_narrow(obj);
            if (CORBA::is_nil(rootContext))
            {
                std::cerr << "Failed to narrow NameService" << std::endl;
                return 1;
            }
        }
        catch (const CORBA::ORB::InvalidName &)
        {
            std::cerr << "Service required is invalid [does not exist]." << std::endl;
            return 1;
        }

        CosNaming::Name contextName;
        contextName.length(1);
        contextName[0].id = (const char *)"ImageService";
        contextName[0].kind = (const char *)"object";

        try
        {
            rootContext->bind(contextName, obj);
        }
        catch (const CosNaming::NamingContext::AlreadyBound &)
        {
            rootContext->rebind(contextName, obj);
        }

        PortableServer        ::POAManager_var pman = poa->the_POAManager();
        pman->activate();

        std::cout << "Server ready and waiting for requests..." << std::endl;

        orb->run();

        orb->destroy();
    }
    catch (const CORBA::Exception &ex)
    {
        std::cerr << "Uncaught CORBA exception: " << ex._name() << std::endl;
        return 1;
    }

    return 0;
}
