#pragma once 

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <cstdlib>
#include <codecvt>
#include <wchar.h>
#include <locale.h>
#include <thread>
#include <functional>
#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <iostream>
#include <condition_variable>

#define STRINGSTREAM 1
#define BYTESTREAM 0

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

#define HOSTIP  "0.0.0.0"
#define HOSTPORT 8765

class WEB 
{
public:
    WEB()
    {
    	run_flag = true;
    	connect_flag = false;
        web_thread = std::make_unique<std::thread>(&WEB::RUN, this);
    }
    virtual ~WEB()
    {
    	std::vector<int> end_data ;
		Push(end_data);
		web_thread->join();
    }
    void Push(std::vector<int>& indexs)
    {
    	if (!connect_flag) return;
        std::unique_lock<std::mutex> lck(web_mutex);
        web_data.push(indexs);
        web_cv.notify_one();
        //std::cout << "----------------------------------------------------" << std::endl;
    }
    void quit()
    {
    	run_flag = false;
    }
private:
    void RUN()
    {
        auto const address = net::ip::make_address(HOSTIP);//绑定ip地址
		auto const port = static_cast<unsigned short>(HOSTPORT);//绑定端口号
		net::io_context ioc{ 1 };
		tcp::acceptor acceptor{ ioc,{ address, port } };
		//while (run_flag)
		//{
			tcp::socket socket{ ioc };
            std::cout << "11--------------------------------------" << std::endl;
			acceptor.accept(socket);
            std::cout << "22--------------------------------------" << std::endl;
            connect_flag = true;
			// 开启线程等待客户端的连接请求

            do_session(socket);
			// std::thread{ std::bind(&do_session,std::move(socket)) }.detach();
		//}

    }

    std::wstring string_to_wstring(const std::string& str)
    {
        std::wstring r;
        const char *source = str.c_str();
        wchar_t *dest = NULL;
        int len = 0;
        int ret = 0;
        len = strlen(source) + 1;
        if(len <= 1)
            return 0;
        dest = new wchar_t[len];
        ret = mbstowcs(dest, source, len);
        r = std::wstring(dest);
        delete[] dest;
        return r;
    }

    std::string wstring_to_string(const std::wstring& ws)
    {
        std::string r = "";
        const wchar_t *source = ws.c_str();
        char *dest = NULL;
        int len = 0;
        int ret = 0;
        len = wcslen(source) + 1;
        if(len <= 1)
            return 0;
        dest = new char[len*sizeof(wchar_t)];
        ret = wcstombs(dest, source, len*sizeof(wchar_t));
        r = std::string(dest);
        delete[] dest;
        return r;
    }

    std::string ansi_to_utf8(const std::string& s)
    {
        static std::wstring_convert<std::codecvt_utf8<wchar_t> > conv;
        return conv.to_bytes(string_to_wstring(s));
    }
    std::string utf8_to_ansi(const std::string& s)
    {
        static std::wstring_convert<std::codecvt_utf8<wchar_t> > conv;
        return wstring_to_string(conv.from_bytes(s));
    }
    
    void do_session(tcp::socket& socket)
    {

        try
        {
            
        	websocket::stream<tcp::socket> ws{ std::move(socket) };
            ws.set_option(websocket::stream_base::decorator(
                [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                    " websocket-server-sync");
            }));
        	ws.accept();//等待客户端连接
        	
            while (run_flag)
            {
            	std::unique_lock<std::mutex> lck(web_mutex);
            	web_cv.wait(lck, [this]{return !web_data.empty();});
                
                std::vector<int> box_indexs = web_data.front();
                web_data.pop();   
                
                if (box_indexs.size() <= 0) break; 
                
#if STRINGSTREAM                
                    std::string data_str;
                    for (int num : box_indexs) {
                        data_str += std::to_string(num) + ",";
                    }
                    // 移除末尾的逗号
                    if (!data_str.empty()) {
                        data_str.pop_back();
                    }

                    // 发送字符串数据
                    ws.write(net::buffer(data_str));   
                    //std::cout << "========================== OK ======================" << box_indexs.size() << std::endl;  
#endif
#if BYTESTREAM
				
				std::vector<uint8_t> binary_data(box_indexs.size() * sizeof(int));
				std::memcpy(binary_data.data(), box_indexs.data(), box_indexs.size()* sizeof(int));
                for (int d : binary_data)
                {
                	std::cout << d << "  ";
                }            
   				std::cout << std::endl;
   				boost::asio::const_buffer buffer(binary_data.data(), binary_data.size() * sizeof(uint8_t));
				ws.write(net::buffer(binary_data)); 
				//ws.write(binary_data.data());
				std::cout << "========================== OK ======================" << std::endl;
#endif 
                
                
                // beast::flat_buffer buffer;// 这个缓冲区将保存传入的消息
                // ws.read(buffer);// 读取一条消息
                // auto out = beast::buffers_to_string(buffer.cdata());
                // std::cout << utf8_to_ansi(out) << std::endl;
    
    
                /*
                // 将读取到的消息再发送回客户端
                ws.text(ws.got_text());
                ws.write(buffer.data());
                */
            } // while
        } /// try
        catch (beast::system_error const& se)
        {
            if (se.code() != websocket::error::closed)
                std::cerr << "!Error: " << se.code().message() << std::endl;
        }
        catch (std::exception const& e)
        {
            std::cerr << "!!!Error: " << e.what() << std::endl;
        }

        std::cout << " BYE " << std::endl;
        connect_flag = false;
    }
private:
    std::unique_ptr<std::thread> web_thread;
    std::queue<std::vector<int>> web_data;
    std::condition_variable web_cv;
    std::mutex web_mutex;
    bool run_flag;
    bool connect_flag;
};
