#include "simulator_plugin.h"

#include <fast_simulator2/world.h>

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/models.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/ros/msg_conversions.h>

// Simulator
#include <fast_simulator2/object.h>

// ----------------------------------------------------------------------------------------------------

void addToUpdateRequest(const sim::ObjectConstPtr& obj, ed::UpdateRequest& req)
{
//    e->setPose(obj->pose());
    req.setType(obj->id(), obj->type());
    req.setShape(obj->id(),  obj->shape());
}

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::SimulatorPlugin() : reconfigured_(false)
{
}

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::~SimulatorPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::configure(tue::Configuration config)
{
    simulator_.configure(config);
    reconfigured_ = true;
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::initialize()
{
    ros::NodeHandle nh;
    ros::AdvertiseServiceOptions opt_set_entity =
            ros::AdvertiseServiceOptions::create<ed::SetEntity>(
                "/ed/sim/set_entity", boost::bind(&SimulatorPlugin::srvSetEntity, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_set_entity_ = nh.advertiseService(opt_set_entity);
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();

    // Step the simulator
    std::vector<sim::ObjectConstPtr> changed_objects;
    simulator_.step(0.01, changed_objects);

    if (reconfigured_)
    {
        // The simulator was reconfigured, so updated all objects

        // Update ED (TODO)
//        const std::map<sim::UUID, sim::ObjectConstPtr>& sim_objects = simulator_.world()->objects;
//        for(std::map<sim::UUID, sim::ObjectConstPtr>::const_iterator it = sim_objects.begin(); it != sim_objects.end(); ++it)
//        {
//            addToUpdateRequest(it->second, req);
//        }

        reconfigured_ = false;
    }
    else
    {
        // Only update all changed objects in the world model
        for(std::vector<sim::ObjectConstPtr>::const_iterator it = changed_objects.begin(); it != changed_objects.end(); ++it)
        {
            addToUpdateRequest(*it, req);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

bool SimulatorPlugin::srvSetEntity(ed::SetEntity::Request& req, ed::SetEntity::Response& res)
{
    if (req.action == ed::SetEntity::Request::ADD)
    {
        ed::models::NewEntityConstPtr e_created = ed::models::create(req.type, tue::Configuration(), req.id);
        if (e_created && e_created->shape)
        {
            // Deserialize pose
            geo::Pose3D pose;
            geo::convert(req.pose, pose);

            update_req_->setShape(req.id, e_created->shape);
            update_req_->setType(req.id, req.type);
            update_req_->setPose(req.id, pose);
        }
        else
        {
            res.error_msg = "No shape could be loaded for type '" + req.type + "'.";
        }
    }
    else if (req.action == ed::SetEntity::Request::DELETE)
    {
        if (world_model_->getEntity(req.id))
        {
            update_req_->removeEntity(req.id);
        }
        else
        {
            res.error_msg = "Entity '" + req.id + "' does not exist.";
        }
    }
    else if (req.action == ed::SetEntity::Request::UPDATE_POSE)
    {
        ed::EntityConstPtr e = world_model_->getEntity(req.id);
        if (e)
        {
            geo::Pose3D new_pose;
            geo::convert(req.pose, new_pose);

            update_req_->setPose(e->id(), new_pose);
        }
        else
        {
            res.error_msg = "Entity '" + req.id + "' does not exist.";
        }
    }

    return true;
}

ED_REGISTER_PLUGIN(SimulatorPlugin)
