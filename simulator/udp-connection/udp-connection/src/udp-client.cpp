#include "udp-client.h"

UdpClient::UdpClient()
{

}

UdpClient::~UdpClient()
{

}

void UdpClient::init(const QString &cfg_path)
{
    load_config(cfg_path);

    clientSocket = new QUdpSocket();

    clientSocket->bind(host, port);

    clientSocket->connectToHost(host, port);

    connect(clientSocket, &QUdpSocket::readyRead,
            this, &UdpClient::readPendingDatagrams);
}

bool UdpClient::isConnected()
{
    if (clientSocket == Q_NULLPTR)
        return false;

    return  clientSocket->state() == QUdpSocket::ConnectedState;
}

void UdpClient::sendData(const QByteArray& data)
{
    clientSocket->writeDatagram(data, host, port);
    clientSocket->flush();
}

void UdpClient::load_config(const QString &path)
{
    CfgReader cfg;
    cfg.load(path);

    QString host_str;
    cfg.getString("UdpClient", "HostAddr", host_str);
    host.setAddress(host_str);

    int port_int;
    cfg.getInt("UdpClient", "Port", port_int);
    port = static_cast<unsigned short>(port_int);
}

void UdpClient::readPendingDatagrams()
{
    while (clientSocket->hasPendingDatagrams())
    {
        QNetworkDatagram recv_data_net = clientSocket->receiveDatagram();

        QByteArray recv_data = recv_data_net.data();

        if (recv_data.isEmpty())
            return;

        if(recv_data.at(0) == 1)
        {
            client_data.deserialize(recv_data);
        }
    }
}
