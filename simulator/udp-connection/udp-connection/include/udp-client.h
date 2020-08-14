//-----------------------------------------------------------------------------
//
//
//
//
//
//-----------------------------------------------------------------------------
/*!
 *  \file
 *  \brief
 *  \copyright
 *  \author
 *  \date
 */

#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <QtGlobal>
#include <QObject>
#include <QDataStream>
#include <QUdpSocket>
#include <QNetworkDatagram>

#include "CfgReader.h"
#include "udp-data-struct.h"

#if defined(UDP_CLIENT_LIB)
# define UDP_CLIENT_EXPORT Q_DECL_EXPORT
#else
# define UDP_CLIENT_EXPORT Q_DECL_IMPORT
#endif

/*!
 *  \class UdpClient
 *  \brief
 */
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
class UDP_CLIENT_EXPORT UdpClient Q_DECL_FINAL : public QObject
{
    Q_OBJECT

public:
    explicit UdpClient();

    ~UdpClient();

    void init(const QString& cfg_path);

    bool isConnected();

    void sendData(const QByteArray &data);

    //
    float getDataTime() {return client_data.time;}

    unsigned int getMsgCount() { return client_data.msgCount; }

    float getVehicleCoord() { return client_data.vehicles.at(0).coord; }

    QString getVehicleRPath() { return client_data.vehicles.at(0).routePath; }

private:
    QUdpSocket *clientSocket;

    udp_server_data_t client_data;

    QHostAddress client_host;
    QHostAddress server_host;

    unsigned short client_port;
    unsigned short server_port;

    void load_config(const QString& path);

public slots:
    void readPendingDatagrams();
};

#endif // UDP_CLIENT_H
