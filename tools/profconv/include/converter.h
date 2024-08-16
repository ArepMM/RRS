//------------------------------------------------------------------------------
//
//      Conversion processor
//      (c) maisvendoo, 20/12/2018
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Conversion processor
 * \copyright maisvendoo
 * \author maisvendoo
 * \date 20/12/2018
 */

#ifndef     CONVERTER_H
#define     CONVERTER_H

#include    <vector>

//#include    "zds-profile-dat-struct.h"
#include    "zds-route-trk-struct.h"
#include    "zds-start-kilometers-dat-struct.h"
#include    "zds-speeds-dat-struct.h"
#include    "zds-svetofor-dat-struct.h"
#include    "zds-branch-tracks-dat-struct.h"
#include    "trajectory_struct.h"
#include    "power_line_element.h"
#include    "neutral_insertion.h"
#include    "cmd-line.h"

#include    <fstream>
#include    <QTextStream>

#define     FILE_TOPOLOGY    std::string("topology.xml")
#define     FILE_START_POINT std::string("waypoint.xml")
#define     DELIMITER_SYMBOL char('\t')
#define     FILE_EXTENTION   std::string(".traj")
#define     FILE_BACKUP_EXTENTION std::string(".bak")

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class ZDSimConverter
{
public:

    ZDSimConverter();

    ~ZDSimConverter();

    int run(int argc, char *argv[]);

private:

    std::string         routeDir;

    std::string         topologyDir;

    std::string         trajectoriesDir;

    zds_trajectory_data_t    tracks_data1;

    zds_trajectory_data_t    tracks_data2;

    zds_start_km_data_t start_km_data;

    zds_speeds_data_t   speeds_data1;

    zds_speeds_data_t   speeds_data2;

    zds_signals_data_t  signals_data1;

    zds_signals_data_t  signals_data2;

    zds_branch_track_data_t branch_track_data1;

    zds_branch_track_data_t branch_track_data2;

    zds_branch_2_2_data_t branch_2minus2_data;

    zds_branch_2_2_data_t branch_2plus2_data;

    route_trajectories_t trajectories1;

    route_trajectories_t trajectories2;

    route_connectors_t split_data1;

    route_connectors_t split_data2;

    route_connectors_t branch_connectors;

    std::vector<power_line_element_t> power_line1;

    std::vector<power_line_element_t> power_line2;

    std::vector<neutral_insertion_t> neutral_insertions;

    CmdLineParseResult parseCommandLine(int argc, char *argv[]);

    QString fileToQString(const std::string &path);

    bool conversion(const std::string &routeDir);

    zds_track_t getNearestTrack(dvec3 point, const zds_trajectory_data_t &tracks_data, float &coord);

    bool readRouteTRK(const std::string &path, zds_trajectory_data_t &track_data);

    bool readRouteTRK(std::ifstream &stream, zds_trajectory_data_t &track_data);

    void writeProfileData(const zds_trajectory_data_t &tracks_data,
                          const std::string &file_name);

    void writeMainTrajectory(const std::string &filename, const zds_trajectory_data_t &tracks_data);

    void createPowerLine(const std::vector<zds_track_t> &tracks_data,
                         std::vector<power_line_element_t> &power_line);

    bool readStartKilometersDAT(const std::string &path, zds_start_km_data_t &waypoints);

    bool readStartKilometersDAT(QTextStream &stream, zds_start_km_data_t &waypoints);

    void writeStations(const std::string &filename, const zds_start_km_data_t &waypoints);

    bool readSpeedsDAT(const std::string &path, zds_speeds_data_t &speeds_data);

    bool readSpeedsDAT(QTextStream &stream, zds_speeds_data_t &speeds_data);

    void writeSpeeds(const std::string &filename, const zds_speeds_data_t &speeds_data);

    bool readSvetoforDAT(const std::string &path, zds_signals_data_t &signals_data);

    bool readSvetoforDAT(QTextStream &stream, zds_signals_data_t &signals_data);

    bool readBranchTracksDAT(const std::string &path, zds_branch_track_data_t &branch_data, const int &dir);

    bool readBranchTracksDAT(QTextStream &stream, zds_branch_track_data_t &branch_data, const int &dir);

    bool checkIsToOtherMain(zds_branch_point_t *branch_point);

    void findFromOtherMain(zds_branch_point_t *branch_point);

    bool calcBranchTrack1(zds_branch_track_t *branch_track);

    bool calcBranchTrack2(zds_branch_track_t *branch_track);

    void calcBranch22(zds_branch_2_2_t *branch22);

    void findSplitsMainTrajectories(const int &dir);

    void addOrCreateSplit(route_connectors_t &split_data, const split_zds_trajectory_t &split_point);

    void splitMainTrajectory(const int &dir);

    void splitAndNameBranch(zds_branch_track_t *branch_track, const int &dir, size_t num_trajectories);

    void nameBranch22(zds_branch_2_2_t *branch_track, const int &dir, size_t num_trajectories);

    bool readRouteMAP(const std::string &path, std::vector<neutral_insertion_t> ni);

    bool findNeutralInsertions(std::ifstream &stream, std::vector<neutral_insertion_t> ni);

    void writeSplits(const route_connectors_t &connectors, const int &dir);

    void writeTopologyTrajectory(const trajectory_t* trajectory);

    void writeTopologyConnectors();

    void writeStartPoints(const zds_start_km_data_t &waypoints);
};

#endif // CONVERTER_H
