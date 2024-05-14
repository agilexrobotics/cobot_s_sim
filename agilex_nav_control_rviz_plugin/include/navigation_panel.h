#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include "control_panel.h"

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QSpinBox>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <mbf_msgs/MoveBaseActionResult.h>
#include <locomotor_msgs/NavigateToPoseActionResult.h>

enum nav_mode
{
    MOVE_BASE,
    MOVE_BASE_FLEX,
    LOCOMOTOR
};

#define USING_NAV_MODE 0

namespace agilex_nav_control_rviz_plugin
{
    // class ControlPanel;

    class AgilexNavigationPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit AgilexNavigationPanel(QWidget *parent = 0);

    public Q_SLOTS:

        void writePose(geometry_msgs::Pose pose);
        void markPose(const geometry_msgs::PoseStamped::ConstPtr &pose);
        void deleteMark();

    protected Q_SLOTS:

        void updateMaxNumGoal(); // update max number of goal
        void initPoseTable();    // initialize the pose table
        void loadGoals();        // load goals data from file
        void saveGoals();        // save goals data to a file

        void singleNavi(); // start navigate from currentRow in poseArray_table_
        void startNavi();  // start navigate for the first pose
        void cancelNavi();

        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose); // goal count sub callback function

        void movebaseStatusCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &result);        // status sub callback function
        void movebaseflexStatusCB(const mbf_msgs::MoveBaseActionResult::ConstPtr &result);          // status sub callback function
        void locomotorStatusCB(const locomotor_msgs::NavigateToPoseActionResult::ConstPtr &result); // status sub callback function

        void checkCycle();

        void checkStatus(actionlib_msgs::GoalStatus status); // check the goal's status
        void pubGoal();

        static void startSpin(); // spin for sub
    protected:
        QSpinBox *maxNumGoal_editor_;
        QPushButton *startNavi_button_, *singleNavi_button_, *cancel_button_;
        QPushButton *reset_button_, *load_button_, *save_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;
        ControlPanel *control_panel_;

        // The ROS node handle.
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_, status_sub_;
        ros::Publisher goal_pub_, cancel_pub_, marker_pub_;

        int maxNumGoal_, cycleCnt_ = 0, curGoalIdx_ = 0;
        bool permit_ = false, cycle_ = false;
        geometry_msgs::PoseArray pose_array_;
        std::string pose_frame_id;
    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
