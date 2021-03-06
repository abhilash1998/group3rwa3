#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <unordered_map>

#include "agility.hpp"
#include "ariac_agv.hpp"
#include "arm.hpp"
#include "gantry.hpp"

#define NUM_AGVS 4
std::vector<std::string> hit_list;
using AriacAgvMap = std::unordered_map<std::string, std::shared_ptr<AriacAgv>>;

namespace {
    std::string build_part_frame(const std::string& product_type, const int camera_index, const int counter)
    {
        return std::string("logical_camera_")
            + std::to_string(camera_index)
            + "_"
            + product_type
            + "_"
            + std::to_string(counter)
            + "_frame"
        ;
    }

    bool does_frame_exist(const std::string& part_in_camera_frame, const double timeout)
    {
        bool rc = true;
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        try {
            tfBuffer.lookupTransform(
                "world",
                part_in_camera_frame,
                ros::Time(0),
                ros::Duration(timeout)
            );
        } catch (tf2::TransformException& ex) {
            rc = false;
        }
        return rc;
    }







    void cater_higher_priority_order_if_necessary(const AriacAgvMap& agv_map,
                                                  AgilityChallenger* const agility,
                                                  Arm* const arm,
                                                  const int current_order_priority);

bool check_pose_orient_of_parts(geometry_msgs::Pose current_pose_in_world
                            ,geometry_msgs::Pose final_pose_in_world){
        
        auto final_orientation = utils::eulerFromQuaternion(final_pose_in_world.orientation.x,final_pose_in_world.orientation.y,final_pose_in_world.orientation.z,final_pose_in_world.orientation.w); //quaternionFromEuler(0, 1.57, 0);
        auto current_orientation = utils::eulerFromQuaternion(current_pose_in_world.orientation.x,current_pose_in_world.orientation.y,current_pose_in_world.orientation.z,current_pose_in_world.orientation.w);

            if (abs(final_pose_in_world.position.x-current_pose_in_world.position.x)<0.3&& (abs(final_pose_in_world.position.y-current_pose_in_world.position.y)<0.3) ){
            if  ((abs(final_orientation[0]-current_orientation[0])<0.1)&& (abs(final_orientation[1]-current_orientation[1])<0.1)&&(abs(final_orientation[2]-current_orientation[2])<0.1)){
                
                ROS_INFO_STREAM("Part placed right");
                return true;
            }
            else{
                ROS_INFO_STREAM("Part placed changing orientation");
                return false;
                
        }
        }
        else{
            ROS_INFO_STREAM("Part placed changing position");
            // ROS_INFO_STREAM(final_pose_in_world<<" goal in tray "<< target_pose_in_world <<" final pose");
            
             return false;

}
                            }



void cater_pose_orient_parts(std::string part_type, Arm*const Arm,
                          std::string camera_frame, 
                          geometry_msgs::Pose goal_in_tray, 
                          std::string agv){
geometry_msgs::Pose target_pose_in_world;
    geometry_msgs::Pose current_pose_in_world;
    
    
     target_pose_in_world = utils::transformToWorldFrame(
            goal_in_tray,
            agv);
    bool flip_=true;
     current_pose_in_world = utils::transformToWorldFrame(camera_frame);
    if (!(check_pose_orient_of_parts( current_pose_in_world
                            , target_pose_in_world))){
            //                        if (pickPart(part_type_name, final_pose_in_world,1)) {
            // placePart(final_pose_in_world, part_pose_in_frame, agv); /// Changed function.
            // }
            if (Arm->pickPart(part_type, current_pose_in_world, 1)) {
        Arm->placePart(current_pose_in_world, target_pose_in_world, agv,flip_);
    }
            cater_pose_orient_parts( part_type,
                             Arm,
                            camera_frame,
                            goal_in_tray,
                            agv);
                                
                            }
}




    void cater_faulty_parts(AgilityChallenger* const agility,
                            Arm* const arm,
                            const std::string& order_id,
                            std::vector<nist_gear::Product>& products)
    {
        // If this part is faulty, move it
        std::string faulty_part_agv_id;
        nist_gear::Product faulty_part_product;
        geometry_msgs::Pose faulty_part_pick_frame;
        while (agility->get_agv_faulty_part(order_id,
                                            faulty_part_agv_id,
                                            faulty_part_product,
                                            faulty_part_pick_frame))
        {
            ROS_INFO_STREAM("Moving faulty part: "
                            << faulty_part_agv_id
                            << ", "
                            << faulty_part_product.type);
            
            // Move this part to the disposal bin
            // TODO: tweak all the stops/sleeps?
            arm->goToPresetLocation(faulty_part_agv_id);
            if (arm->pickPart(faulty_part_product.type, faulty_part_pick_frame, 1))
            {
                ros::Duration(2.0).sleep();
                arm->goToPresetLocation(faulty_part_agv_id);
                arm->goToPresetLocation("home2");
                ros::Duration(0.5).sleep();
                arm->deactivateGripper();

                // Place this product back in the queue to be
                // picked again elsewhere
                products.push_back(faulty_part_product);
            }
            else
            {
                ROS_ERROR_STREAM("Failed to pick the faulty part");
            }
        }
    }


void cater_flip_parts(std::string part_type, Arm*const Arm,
                          std::string camera_frame, 
                          geometry_msgs::Pose goal_in_tray, 
                          std::string agv)
    {
bool flip_=true;
    auto target_pose_in_world = utils::transformToWorldFrame(
        goal_in_tray,
        agv);
        // ROS_INFO_WARN()
    auto init_pose_in_world = utils::transformToWorldFrame(camera_frame);
    auto target_pose_in_world_euler= utils::eulerFromQuaternion(target_pose_in_world.orientation.x,
                                    target_pose_in_world.orientation.y,
                                    target_pose_in_world.orientation.z,
                                    target_pose_in_world.orientation.w);
    // float pi=0;
    float pi=22/7;
    target_pose_in_world_euler[0]=target_pose_in_world_euler[0]-pi/2;
    auto target_pose_in_world_quaternion=utils::quaternionFromEuler(target_pose_in_world_euler[0],target_pose_in_world_euler[1],target_pose_in_world_euler[2]);

    
     
    // goal_in_tray.orientation=goal_in_tray_quaternion;

    target_pose_in_world.orientation.x= target_pose_in_world_quaternion[0];
    target_pose_in_world.orientation.y= target_pose_in_world_quaternion[1];
    target_pose_in_world.orientation.z= target_pose_in_world_quaternion[2];
    target_pose_in_world.orientation.w= target_pose_in_world_quaternion[3];

    // auto init_in_frame_euler=utils ::eulerFromQuaternion( init_pose_in_world.orientation.x);
    // Arm->goToPresetLocation(agv);
            if (Arm->pickPart(part_type, init_pose_in_world , 1))
            {
                // (geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv,bool flip_);
                Arm->placePart(init_pose_in_world,target_pose_in_world,agv,flip_);
            }
}





bool check_for_flip_part(std::string part_type,
                         std::string camera_frame,
                         geometry_msgs::Pose goal_in_tray_frame, 
                         std::string agv){
    auto target_pose_in_world = utils::transformToWorldFrame(
        goal_in_tray_frame,
        agv);
    auto target_pose_in_euler= utils::eulerFromQuaternion(target_pose_in_world.orientation.x,
                                    target_pose_in_world.orientation.y,
                                    target_pose_in_world.orientation.z,
                                    target_pose_in_world.orientation.w);
    auto init_pose_in_world = utils::transformToWorldFrame(camera_frame);
    auto init_pose_in_world_euler = utils::eulerFromQuaternion( init_pose_in_world.orientation.x,init_pose_in_world.orientation.y,init_pose_in_world.orientation.z,init_pose_in_world.orientation.w);

    if((abs(init_pose_in_world_euler[0] - target_pose_in_euler[0])<3.16) && (abs(init_pose_in_world_euler[0] - target_pose_in_euler[0])>3.13)){
        return true;

    }
    else{
        return false;
    }
    
}



    void cater_kitting_shipments(const AriacAgvMap& agv_map,
                                 AgilityChallenger* const agility,
                                 Arm* const arm,
                                 const int order_priority,
                                 const std::string& order_id,
                                 std::vector<nist_gear::KittingShipment>& kitting_shipments)
    {
        // Ignore request if there are no kitting shipments
        if (kitting_shipments.empty())
        {
            return;
        }

        ROS_INFO_STREAM("Catering "
                        << kitting_shipments.size()
                        << " kitting shipments from order with priority of "
                        << order_priority);

        int counter = 0;
        // parse each kitting shipment
        for (const auto& ks : kitting_shipments)
        {
            std::vector<nist_gear::Product> products = ks.products;
            if (products.empty())
            {
                ROS_FATAL_STREAM("Kitting shipment had no products?");
                ros::shutdown();
                return;
            }

            // loop through each product in this shipment
            while (!products.empty())
            {
                // Remove this product from the list, with the intention that
                // it will be catered to
                const nist_gear::Product product = products.front();
                products.erase(products.begin());
                ROS_INFO_STREAM("Catering product '"
                                << product.type
                                << "', "
                                << products.size()
                                << " remaining afterwards");

                // Get the bins in which this part appears
                const std::vector<int> bin_indices = agility->get_camera_indices_of(product.type);
                if (bin_indices.empty())
                {
                    ROS_FATAL_STREAM(
                        "No matching part '"
                        << product.type
                        << "' found by any logical camera with contents "
                        << agility->get_logical_camera_contents()
                    );
                    // ros::shutdown();
                    // return;
                }

                // counter++;

                for (auto iter = bin_indices.cbegin(); iter != bin_indices.cend(); ++iter)
                {
                    std::string part_frame;
                    for (counter = 1; counter <= 12; counter++)
                    {
                        bool hit = false;
                        part_frame = build_part_frame(product.type, *iter, counter);
                        if (does_frame_exist(part_frame, 0.5))
                        {
                            if (hit_list.empty())
                            {   
                                // ROS_INFO_STREAM("Hit List Updated");
                                hit_list.emplace_back(product.type + std::to_string(counter));
                            }
                            else
                            {
                                for (auto s : hit_list)
                                {
                                    if(s.compare(product.type + std::to_string(counter)) == 0)
                                    {
                                        hit = true;
                                        break;
                                    }
                                }
                            }
                            if (hit) {continue;}
                            else 
                            {
                                hit_list.emplace_back(product.type + std::to_string(counter));
                                break;
                            }
                        }
                    }
                    if (!does_frame_exist(part_frame, 0.5))
                    {
                        continue;
                    }

                    // Move the part from where it is to the AGV bed
                    ROS_INFO_STREAM("Moving part '" << product.type << "' to '" << ks.agv_id << "' (" << part_frame << ")");

                    // product.pose
                    bool flip_ =check_for_flip_part(product.type,part_frame,product.pose,ks.agv_id);




                    if (flip_){
                             ROS_WARN_STREAM("flip part");
                            //  flip_=true;
                            // ros::Duration(1).sleep();
                            
                            cater_flip_parts(product.type,arm,part_frame,product.pose, ks.agv_id);
                         }
                    else{
                        ROS_WARN_STREAM("No Flip");
                        arm->movePart(product.type, part_frame, product.pose, ks.agv_id,flip_);
                    ROS_INFO_STREAM("Placed part '" << product.type << "' at '" << ks.agv_id << "'");
                    }
                


                    
                    agility->queue_for_fault_verification(
                        product,
                        order_id,
                        ks.agv_id,
                        arm->transform_to_world_frame(product.pose, ks.agv_id)
                    );
                    ros::Duration(0.5).sleep();

                    // Give an opportunity for higher priority orders
                    cater_higher_priority_order_if_necessary(agv_map, agility, arm, order_priority);

                    // If there is no sensor blackout, check for faulty parts
                    if (!agility->is_sensor_blackout_active())
                    {
                        // After checking, give an opportunity for higher
                        // priority orders
                        cater_faulty_parts(agility, arm, order_id, products);
                        cater_higher_priority_order_if_necessary(agv_map, agility, arm, order_priority);
cater_pose_orient_parts(product.type,  arm,
                           part_frame, 
                          product.pose, 
                          ks.agv_id);
                    }
                    else
                    {
                         ROS_ERROR_STREAM("SENSOR BLCKOUT");
                    }

                    // It may be faulty, but we placed the part. Whether it was
                    // already declared faulty and moved, or there was a sensor
                    // blackout, we're done with the product for now.
                    break;
                }

                // If we have finished placing all of the parts but parts still
                // need verification for faults, wait here until the sensor
                // blackout is done, then cater them. If any of them are
                // faulty, it'll add the product back into the products vector.
                if (products.empty() && agility->needs_fault_verification(ks.agv_id))
                {
                    ROS_INFO_STREAM("Waiting for sensor blackout to finish...");
                    do {
                        static ros::Duration d(0.1);
                        d.sleep();
                    } while (agility->needs_fault_verification(ks.agv_id));

                    cater_faulty_parts(agility, arm, order_id, products);

                    // If products were added back, first give an opportunity
                    // for higher priority orders before resuming
                    if (!products.empty())
                    {
                        cater_higher_priority_order_if_necessary(agv_map, agility, arm, order_priority);
                    }
                }
            }

            // If we're here, we have placed all products in this shipment
            ros::Duration(1.0).sleep();
            const auto agv_iter = agv_map.find(ks.agv_id);
            if (agv_map.cend() != agv_iter)
            {
                const std::shared_ptr<AriacAgv> agv = agv_iter->second;
                if (agv->is_ready_to_deliver())
                {
                    agv->submit_shipment(
                        ks.station_id,
                        ks.shipment_type
                    );
                    ROS_INFO_STREAM("Submitted AGV with ID " << ks.agv_id);
                }
                else
                {
                    ROS_ERROR_STREAM("AGV with ID " << ks.agv_id << " is not ready to ship");
                }
            }
            else
            {
                ROS_FATAL_STREAM("Unknown AGV with ID " << ks.agv_id);
                ros::shutdown();
                return;
            }
        }
    }

    void cater_order(const AriacAgvMap& agv_map,
                     AgilityChallenger* const agility,
                     Arm* const arm,
                     const int order_priority,
                     nist_gear::Order& order)
    {
        cater_kitting_shipments(
            agv_map,
            agility,
            arm,
            order_priority,
            order.order_id,
            order.kitting_shipments
        );
    }

    void cater_higher_priority_order_if_necessary(const AriacAgvMap& agv_map,
                                                  AgilityChallenger* const agility,
                                                  Arm* const arm,
                                                  const int current_order_priority)
    {
        if (agility->higher_priority_order_requested(current_order_priority))
        {
            ROS_INFO_STREAM("Catering higher priority order...");
            int new_order_priority;
            nist_gear::Order new_order;
            new_order_priority = agility->consume_pending_order(new_order);
            cater_order(agv_map, agility, arm, new_order_priority, new_order);
            ROS_INFO_STREAM("Finished higher priority order, returning to previous order");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    // Create interfaces to each of the AGVs
    AriacAgvMap agv_map;
    for (int i = 0; i < NUM_AGVS; i++)
    {
        // AGV topics use identifiers in the range [1,NUM_AGVS], but this loop
        // is [0,NUM_AGVS-1], so add 1 to all indices when creating them here
        auto agv = std::shared_ptr<AriacAgv>(new AriacAgv(&nh, i+1));
        agv_map[agv->get_id()] = agv;
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();

    AgilityChallenger agility(&nh);
    Arm arm;

    //
    // Start the competition
    //

    ros::Duration wait_for_competition_state(0.1);
    bool competition_state_valid = false;
    std::string competition_state;
    ros::Subscriber competition_state_sub = nh.subscribe<std_msgs::String>(
        "/ariac/competition_state",
        1,
        [&](const std_msgs::String::ConstPtr& msg) 
        {
            competition_state = msg->data;
            competition_state_valid = true;
        }
    );

    // Spin the node until we've collected the competition state
    while (!competition_state_valid)
    {
        wait_for_competition_state.sleep();
        ros::spinOnce();
    }

    // If the competition has not started, we place a request to start it
    if (competition_state == "go")
    {
        ROS_INFO_STREAM("Competition is already started.");
    }
    else
    {
        // Create the client
        assert(competition_state == "init");
        ros::ServiceClient start_competition_scl = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // Wait a very long time, because this service server may take a while
        // to be created
        assert(start_competition_scl.waitForExistence(ros::Duration(30.0)));

        // Place the request
        std_srvs::Trigger start_competition_srv;
        if (!(start_competition_scl.call(start_competition_srv) && start_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to start the competition: '"<< start_competition_srv.response.message);
            return 1;
        }

        // Wait a little and collect the updated competition state
        competition_state_valid = false;
        ros::Duration(0.25).sleep();
        ros::spinOnce();
        if (competition_state == "go")
        {
            ROS_INFO_STREAM("Competition was started.");
        }
        else
        {
            ROS_ERROR_STREAM("Competition state did not update as expected: '"<< competition_state);
            return 2;
        }
    }

    //
    // If we're here, the competition has started
    //

    arm.goToPresetLocation("home1");
    arm.goToPresetLocation("home2");

    int current_order_priority;
    nist_gear::Order current_order;
    ros::Duration rate(0.1);
    while (ros::ok())
    {
        current_order_priority = agility.consume_pending_order(current_order);
        if (0 != current_order_priority)
        {
            cater_order(
                agv_map,
                &agility,
                &arm,
                current_order_priority,
                current_order
            );
        }
        else
        {
            rate.sleep();
        }
    }

    return 0;
}
