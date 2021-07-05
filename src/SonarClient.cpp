#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

SonarClient::SonarClient(boost::asio::io_service& service,
                         const Duration& checkerPeriod) :
    socket_(service),
    remote_(),
    sonarId_(0),
    checkerPeriod_(checkerPeriod),
    checkerTimer_(service, checkerPeriod_),
    statusListener_(service),
    statusCallbackId_(0),
    data_(0)
{
    std::memset(&initialHeader_, 0, sizeof(initialHeader_));

    this->initiate_connection();

    this->checkerTimer_.async_wait(
        std::bind(&SonarClient::checker_callback, this, std::placeholders::_1));
}

bool SonarClient::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sonarId_;
}

/**
 * Connection Watchdog.
 *
 * This is an asynchronous loop independent from the connection status (it is
 * looping even if there is no connection). It checks the status of the
 * connection with the sonar and restarts it if necessary.
 */
void SonarClient::checker_callback(const boost::system::error_code& err)
{
    std::cout << "Checking" << std::endl;
    if(!socket_.is_open()) {
        std::cout << "Not connected" << std::endl << std::flush;
    }
    this->checkerTimer_.expires_from_now(checkerPeriod_);
    this->checkerTimer_.async_wait(
        std::bind(&SonarClient::checker_callback, this, std::placeholders::_1));
}

void SonarClient::check_reception(const boost::system::error_code& err)
{
    // no real handling for now
    if(err) {
        std::ostringstream oss;
        oss << "oculue::SonarClient, reception error : " << err;
        throw oss.str();
    }
}

void SonarClient::initiate_connection()
{
    statusCallbackId_ = statusListener_.add_callback(&SonarClient::on_first_status, this);
}

void SonarClient::on_first_status(const OculusStatusMsg& msg)
{
    // got a status message. No need to keep listening.
    statusListener_.remove_callback(statusCallbackId_);
    
    // device id and ip fetched from status message
    sonarId_ = msg.hdr.srcDeviceId;
    remote_ = remote_from_status<EndPoint>(msg);

    // attempting connection
    socket_.async_connect(remote_, boost::bind(&SonarClient::on_connect, this, _1));
}

void SonarClient::on_connect(const boost::system::error_code& err)
{
    if(err) {
        std::ostringstream oss;
        oss << "oculus::SonarClient : connection failure. ( " << remote_ << ")";
        throw std::runtime_error(oss.str());
    }
    std::cout << "Connection successful (" << remote_ << ")" << std::endl << std::flush;
    
    // this enters the ping data reception loop
    this->initiate_receive();

    this->on_connect();
}

void SonarClient::on_connect()
{}

void SonarClient::initiate_receive()
{
    // asynchronously scan input until finding a valid header.
    // /!\ To be checked : This function and its callback handle the data
    // synchronization with the start of the ping message. It is relying on the
    // assumption that if the data left in the socket is less than the size of
    // the header, a short read will happen, so that the next read will be
    // exactly aligned on the next header.
    static unsigned int count = 0;
    //std::cout << "Initiate receive : " << count << std::endl << std::flush;
    count++;
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 
                            sizeof(initialHeader_)),
        boost::bind(&SonarClient::receive_callback, this, _1, _2));
}

void SonarClient::receive_callback(const boost::system::error_code err,
                                   std::size_t receivedByteCount)
{

    static unsigned int count = 0;
    //std::cout << "Receive callback : " << count << std::endl << std::flush;
    count++;
    // This function received only enough bytes for an OculusMessageHeader.  If
    // the header is valid, the control is dispatched to the next state
    // depending on the header content (message type). For now only simple ping
    // is implemented, but it seems to be the only message sent by the Oculus.
    // (TODO : check this last statement. Checked : wrong. Other messages seems
    // to be sent but are not documented).
    this->check_reception(err);
    if(receivedByteCount != sizeof(initialHeader_) || !this->is_valid(initialHeader_)) {
        // Either we got data in the middle of a ping or did not get enougth
        // bytes (end of message). Continue listening to get a valid header.
        //std::cout << "Header reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }

    // Messsage header is valid. Now getting the remaining part of the message.
    // (The header contains the payload size, we can receive everything and
    // parse afterwards).  This performs a synchonous read on the socket (block
    // until all is received) (TODO check the timeout)
    data_.resize(sizeof(initialHeader_) + initialHeader_.payloadSize);
    auto byteCount = boost::asio::read(socket_,
        boost::asio::buffer(data_.data() + sizeof(initialHeader_), initialHeader_.payloadSize));
    if(byteCount != initialHeader_.payloadSize) {
        // We did not get enough bytes. Reinitiating reception.
        std::cout << "Data reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }

    // We did received everything. copying initial header in data_ to have a
    // full message, then dispatching.
    *(reinterpret_cast<OculusMessageHeader*>(data_.data())) = initialHeader_;
    
    // handle message is to be reimplemented in a subclass
    this->handle_message(initialHeader_, data_);

    // Continuing the reception loop.
    //std::cout << "Looping" << std::endl << std::flush;
    this->initiate_receive();
   
}

void SonarClient::handle_message(const OculusMessageHeader& header,
                                 const std::vector<uint8_t>& data)
{}

}; //namespace oculus
}; //namespace narval

