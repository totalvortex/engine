#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
using boost::asio::ip::tcp;

enum { max_length = 1024 };



int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: "<< argv[01]<<" <host> <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    tcp::socket s(io_service);
    tcp::resolver resolver(io_service);
    boost::asio::connect(s, resolver.resolve({argv[1], argv[2]}));

    std::cout << "Enter message: ";
    char request[max_length];
    std::cin.getline(request, max_length);
    size_t request_length = std::strlen(request);
    boost::asio::write(s, boost::asio::buffer(request, request_length));

    char reply[max_length];
    size_t reply_length = boost::asio::read(s,
        boost::asio::buffer(reply, request_length));
    std::cout << "Reply is: ";
    std::cout.write(reply, reply_length);
    std::cout << "\"\n";
    std::cout << sizeof(float);
 //   float msg[16];


    using boost::lexical_cast;
    using boost::bad_lexical_cast;



    std::vector<float> msg;

   for(int i=0;i<max_length;i+=4)
    {
      float *p=(float *)&reply[i];
        try
        {
            msg.push_back(lexical_cast<float>(p));
        }
        catch(bad_lexical_cast &)
        {
            msg.push_back(0);
        }
    }





      //  std::string outbound_data_ = archive_stream.str();
    std::cout << "(" << msg[0] <<", " << msg[1] <<", " << msg[2]<< ")" << std::endl ;

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
/*
    std::ostringstream archive_stream;
        boost::archive::text_oarchive archive(archive_stream);
        archive << matrix;
        
        std::string outbound_data_ = archive_stream.str();
         boost::asio::write(*sock, boost::asio::buffer(outbound_data_, outbound_data_.length()));*/