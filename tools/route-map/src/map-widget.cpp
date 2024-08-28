#include    <map-widget.h>
#include    <QPainter>
#include    <QWheelEvent>
#include    <connector.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
MapWidget::MapWidget(QWidget *parent) : QWidget(parent)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
MapWidget::~MapWidget()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::resize(int width, int height)
{
    QWidget::resize(width, height);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::paintEvent(QPaintEvent *event)
{
    if (traj_list == Q_NULLPTR)
    {
        return;
    }

    if (conn_list == Q_NULLPTR)
    {
        return;
    }

    if (train_data == Q_NULLPTR)
    {
        return;
    }

    for (auto traj : *traj_list)
    {
        drawTrajectory(traj);
    }

    drawConnectors(conn_list);

    drawTrain(train_data);

    if (stations == Q_NULLPTR)
    {
        return;
    }

    drawStations(stations);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawTrajectory(Trajectory *traj)
{
    QPainter painter;
    painter.begin(this);

    for (auto track : traj->getTracks())
    {
        QPoint p0 = coord_transform(track.begin_point);
        QPoint p1 = coord_transform(track.end_point);

        painter.drawLine(p0, p1);
    }

    painter.end();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawTrain(tcp_simulator_update_t *train_data)
{
    for (size_t i = 0; i < train_data->vehicles.size(); ++i)
    {
        if (i == train_data->controlled_vehicle)
        {
            QColor color(192, 64, 64);
            drawVehicle(train_data->vehicles[i],color);
            continue;
        }
        if (i == train_data->current_vehicle)
        {
            QColor color(192, 192, 0);
            drawVehicle(train_data->vehicles[i],color);
            continue;
        }
        QColor color(64, 128, 0);
        drawVehicle(train_data->vehicles[i],color);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawVehicle(simulator_vehicle_update_t &vehicle, QColor color)
{
    QPen pen;
    pen.setWidth(5 + std::floor(scale));
    pen.setColor(color);
    pen.setCapStyle(Qt::FlatCap);

    QPainter p;
    p.begin(this);
    p.setPen(pen);

    dvec3 fwd;
    fwd.x = vehicle.position_x + vehicle.orth_x * (vehicle.length / 2.0 - 0.45);
    fwd.y = vehicle.position_y + vehicle.orth_y * (vehicle.length / 2.0 - 0.45);
    fwd.z = 0;

    dvec3 bwd;
    bwd.x = vehicle.position_x - vehicle.orth_x * (vehicle.length / 2.0 - 0.45);
    bwd.y = vehicle.position_y - vehicle.orth_y * (vehicle.length / 2.0 - 0.45);
    bwd.z = 0;

    QPoint fwd_point = coord_transform(fwd);
    QPoint bwd_point = coord_transform(bwd);

    p.drawLine(fwd_point, bwd_point);

    p.end();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawConnectors(conn_list_t *conn_list)
{
    for (auto conn : *conn_list)
    {
        drawConnector(conn);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawConnector(Connector *conn)
{
    if (conn == Q_NULLPTR)
    {
        return;
    }

    Trajectory *fwd_traj = conn->getFwdTraj();
    Trajectory *bwd_traj = conn->getBwdTraj();

    if ( (fwd_traj == Q_NULLPTR) || (bwd_traj == Q_NULLPTR) )
    {
        return;
    }

    if ( (fwd_traj->getTracks().size() == 0) || (fwd_traj->getTracks().size() == 0) )
    {
        return;
    }

    double conn_length = 15.0;

    track_t fwd_track = fwd_traj->getFirstTrack();
    dvec3 center = fwd_track.begin_point;
    dvec3 fwd = center + fwd_track.orth * conn_length;

    track_t bwd_track = bwd_traj->getLastTrack();
    dvec3 bwd = center - bwd_track.orth * conn_length;

    QPen pen;
    pen.setWidth(2 + std::floor(sqrt(scale)));
    pen.setColor(QColor(0, 0, 128));

    QPainter painter;
    painter.begin(this);
    painter.setPen(pen);

    QPoint center_point = coord_transform(center);
    QPoint fwd_point = coord_transform(fwd);
    QPoint bwd_point = coord_transform(bwd);

    painter.drawLine(center_point, fwd_point);
    painter.drawLine(center_point, bwd_point);

    SwitchLabel * sw_label = switch_labels.value(conn->getName(), Q_NULLPTR);

    if (sw_label != Q_NULLPTR)
    {
        sw_label->move(center_point);
        sw_label->show();
    }

    painter.end();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::drawStations(topology_stations_list_t *stations)
{
    for (auto station : *stations)
    {
        QPainter painter;
        painter.begin(this);
        QFont font("Arial", 14);
        painter.setFont(font);

        dvec3 station_place{station.pos_x, station.pos_y, station.pos_z};
        QPoint place = coord_transform(station_place);

        painter.drawText(place, station.name);

        painter.end();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QPoint MapWidget::coord_transform(dvec3 point)
{
    QPoint p;

    if (folow_vehicle)
    {
        int curr = train_data->current_vehicle;

        map_shift.setX(- train_data->vehicles[curr].position_y * scale);
        map_shift.setY(- train_data->vehicles[curr].position_x * scale);
    }

    p.setX(this->width() / 2 + map_shift.x() + scale * point.y);
    p.setY(this->height() / 2 + map_shift.y() + scale * point.x);

    return p;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::wheelEvent(QWheelEvent *event)
{
    if ((event->angleDelta().y() > 0) && (scale < 16.0))
    {
        scale *= scale_inc_step_coeff;
        map_shift = map_shift * scale_inc_step_coeff;
        prev_map_shift = prev_map_shift * scale_inc_step_coeff;
    }

    if ((event->angleDelta().y() < 0) && (scale > 0.25))
    {
        scale *= scale_dec_step_coeff;
        map_shift = map_shift * scale_dec_step_coeff;
        prev_map_shift = prev_map_shift * scale_dec_step_coeff;
    }

    event->accept();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton)
    {
        map_shift = prev_map_shift + event->pos() - mouse_pos;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (folow_vehicle)
        {
            folow_vehicle = false;
            prev_map_shift = map_shift;
        }
        mouse_pos = event->pos();
    }

    if (event->button() == Qt::MiddleButton)
    {
        folow_vehicle = true;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        prev_map_shift = map_shift;
    }
}