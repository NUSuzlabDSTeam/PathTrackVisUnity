

// UDP Transceiver


#pragma once

#include <map>

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/system/error_code.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/lambda/bind.hpp>

#include <boost/algorithm/string.hpp>

void dummy_handler(
  const boost::system::error_code& error, // Result of operation.
  std::size_t bytes_transferred           // Number of bytes sent.
  );

class UDPSender
{

	boost::asio::io_service m_ios;
	boost::system::error_code m_lasterror;

	boost::asio::ip::udp::socket m_socket;
	boost::asio::ip::udp::endpoint m_target_ep;

	std::unique_ptr<boost::thread> m_th;

public:
	bool m_is_broadcasting;
	std::string m_ip_address;
	int m_port;

	UDPSender() : m_socket( m_ios )
	{
	}

	bool isOpen()
	{
		return m_socket.is_open();
	}

	void Init( int port, bool broadcasting = false)
	{
		m_port = port;
		m_ip_address = "192.168.10.4";
		m_is_broadcasting = true;
		Init( m_ip_address, port, broadcasting);
	}

	void Init(  std::string ip_address, int port, bool broadcasting = false )
	{
		m_port = port;
		m_ip_address = ip_address;
		m_is_broadcasting = broadcasting;
	}

	void Open(std::string ip_address, int port, bool broadcasting = false)
	{
		Init( ip_address, port, broadcasting);
		OpenImpl();
	}

	void Open(int port, bool broadcasting = false){
		Init(port, broadcasting);
		OpenImpl();
	}

	void OpenImpl()
	{
		m_socket.open(boost::asio::ip::udp::v4(), m_lasterror);

		if (!m_lasterror)
		{
			if( m_is_broadcasting )
			{
				m_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
				m_socket.set_option(boost::asio::socket_base::broadcast(true));
				m_target_ep.address(boost::asio::ip::address_v4::broadcast());
			}else{
				m_target_ep.address(boost::asio::ip::address::from_string( m_ip_address ) );
			}
			m_target_ep.port(m_port);

			// run thread
			m_th.reset( 
				new boost::thread( 
					boost::bind(  &boost::asio::io_service::run, &m_ios)
						)
				) ;	
		}
	}

	void Close()
	{
		m_socket.close(m_lasterror);
		m_ios.stop();
	}

	void SendBytes( const void* data, size_t len)
	{
		//m_socket.async_send_to(  boost::asio::buffer( data, len ), m_target_ep, 
		//	[](	const boost::system::error_code& error, // Result of operation.
		//		std::size_t bytes_transferred   ) // à¯êîÇÃíËã`
		//	{       // ä÷êîñ{ëÃ
		//	}
		//);

		//m_ios.run();

		//m_socket.async_send_to(
		//	boost::asio::buffer( data, len ), 
		//	m_target_ep , 
		//	&dummy_handler
		//);

				m_socket.send_to(
			boost::asio::buffer( data, len ), 
			m_target_ep 
		);
	}
};

//template< int N = 2048>
class UDPReceiver
{
	static const int N = 4096;
public:
	typedef boost::array<char, N> buffer_type;
	typedef boost::function< void(buffer_type&, size_t) > callback_type;

	buffer_type m_buf;
	size_t m_received_len;

private:
	boost::asio::io_service m_ios;
	boost::shared_ptr<boost::asio::ip::udp::socket> m_socket;

	const std::string listen_address = "0.0.0.0";

	callback_type m_callback;

	boost::system::error_code m_lasterror;

	boost::asio::ip::udp::endpoint m_local_ep;
	boost::asio::ip::udp::endpoint m_remote_ep;

	std::unique_ptr<boost::thread> m_th;
	boost::thread th;

public:
	

	//std::string m_ip_address;
	int m_port;

	UDPReceiver() : m_socket()
	{
	}

	virtual ~UDPReceiver()
	{
		Close();
	}

	void Init( int port )
	{
		m_port = port;
	}

	void Open( int port)
	{
		Init( port);
		OpenImpl();
	}

	void MultiCastOpen(const std::string multicast_address,	const unsigned short multicast_port) {
		// Create the socket so that multiple may be bound to the same address.
		m_socket.reset(new boost::asio::ip::udp::socket(m_ios));
		boost::asio::ip::udp::endpoint listen_endpoint(
			boost::asio::ip::address::from_string(listen_address), multicast_port);
		m_socket->open(listen_endpoint.protocol());
		m_socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
		m_socket->bind(listen_endpoint);

		// Join the multicast group.
		m_socket->set_option(
			boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(multicast_address)));
		OpenImpl(false);
	}

	void OpenImpl(bool isReseted = true)
	{
		if (isReseted) {
			m_socket.reset(
				new boost::asio::ip::udp::socket(m_ios, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_port))
			);
		}
		//m_socket->open(boost::asio::ip::udp::v4(), m_lasterror);
		
		if (!m_lasterror)
		{
			invoke_async_call();
			m_th.reset( 
				new boost::thread( 
					boost::bind(  &boost::asio::io_service::run, &m_ios)
						)
				) ;				

		}
	}

	void Close()
	{
		m_ios.stop();
		m_callback.clear();
		if( m_socket.get() != NULL)
		{
			m_socket->cancel();
			m_socket.reset();
		}
		if( m_th.get() != NULL)
			m_th->join();
		//m_socket->close(m_lasterror);
		//m_socket.reset();

	}

	void invoke_async_call()
	{
		if( m_socket )
		{
			m_socket->async_receive(  
				boost::asio::buffer( m_buf ), 
				0,
				boost::bind( &UDPReceiver::receiver_func, this, boost::lambda::_1, boost::lambda::_2)
				);
		}
	}

	virtual void receiver_func( const boost::system::error_code &e, size_t len )
	{
		if (e == boost::asio::error::operation_aborted)
		{
			std::cout << "Receive callback aborted." << std::endl;
			return;
		}

		//std::cout << "[recv n=" << len << "]" ;
		//std::cout.write( m_buf.data(), len);
		
		if( m_callback )
		{
			m_callback( m_buf, len );
		}

		invoke_async_call();
	}

	void set_callback( callback_type callback )
	{
		m_callback = callback;
	}

};


class FrameReceiver : public UDPReceiver
{
public:
	typedef std::map< std::string, std::string> map_type;
	map_type params;

	FrameReceiver()
	{
		set_callback(
			boost::bind( &FrameReceiver::recv_cb, this, _1, _2)
			);
	}
		
	void recv_cb( UDPReceiver::buffer_type buf, size_t len )
	{
		// received string
		std::string str( buf.begin(), buf.begin() + len);

		// parse
		std::vector<std::string> records;
		// split by crlf
		boost::algorithm::split( records, str, boost::is_any_of("\n"), boost::algorithm::token_compress_mode_type::token_compress_on);

		for each( const std::string &record in records)
		{
			if(record.empty() || record == "")continue;
			std::vector<std::string> term;
			boost::algorithm::split( term, record, boost::is_any_of(":"), boost::algorithm::token_compress_mode_type::token_compress_on);
			if( term.size() == 2 )
			{
				params[term[0]] = term[1];
			}else{
				std::cout << "received record is illigal" << record << std::endl;
			}
		}
	}
};





