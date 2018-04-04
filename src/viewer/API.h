#pragma once

#include <vision/AIVisionClient.h>
#include <json/json.h>
#include <string>
#include <QObject>
#include <Data.h>

class API : public QObject
{
    Q_OBJECT

public:
    API(bool simulation, RhobanSSL::AIVisionClient::Team team);
    virtual ~API();

    bool simulation;

signals:

public slots:
    bool isSimulation();
    bool isYellow();

    bool hasVisionData();
    unsigned int visionPackets();

protected:
    RhobanSSL::Data data;
    RhobanSSL::AIVisionClient::Team team;
    RhobanSSL::AIVisionClient visionClient;
};
