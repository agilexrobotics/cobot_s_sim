#include <QTextEdit>
#include <QFile>
#include <QFileDialog>
#include <QPainter>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

#include "navigation_panel.h"
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <mbf_msgs/MoveBaseActionGoal.h>
#include <locomotor_msgs/NavigateToPoseActionGoal.h>

namespace agilex_nav_control_rviz_plugin
{

    AgilexNavigationPanel::AgilexNavigationPanel(QWidget *parent)
        : rviz::Panel(parent), nh_(), maxNumGoal_(1), pose_frame_id("map")
    {
#if USING_NAV_MODE == 0
        ROS_INFO("Using MOVE_BASE navigation mode");
        status_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 1,
                                                                          boost::bind(&AgilexNavigationPanel::movebaseStatusCB, this, _1));
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
#elif USING_NAV_MODE == 1
        ROS_INFO("Using MOVE_BASE_FLEX navigation mode");
        status_sub_ = nh_.subscribe<mbf_msgs::MoveBaseActionResult>("/move_base_flex/move_base/result", 1,
                                                                    boost::bind(&AgilexNavigationPanel::movebaseflexStatusCB, this, _1));
        goal_pub_ = nh_.advertise<mbf_msgs::MoveBaseActionGoal>("/move_base_flex/move_base/goal", 1);
        cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base_flex/move_base/cancel", 1);
#else
        ROS_INFO("Using LOCOMOTOR navigation mode");
        status_sub_ = nh_.subscribe<locomotor_msgs::NavigateToPoseActionResult>("/locomotor/navigate/result", 1,
                                                                                boost::bind(&AgilexNavigationPanel::locomotorStatusCB, this, _1));
        goal_pub_ = nh_.advertise<locomotor_msgs::NavigateToPoseActionGoal>("/locomotor/navigate/goal", 1);
        cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/locomotor/navigate/cancel", 1);
#endif

        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 1,
                                                              boost::bind(&AgilexNavigationPanel::goalCntCB, this, _1));
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        QVBoxLayout *root_layout = new QVBoxLayout;
        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        maxNumGoal_layout->addWidget(new QLabel("目标最大数量："));
        maxNumGoal_editor_ = new QSpinBox;
        maxNumGoal_editor_->setRange(1, 10);
        maxNumGoal_editor_->setSingleStep(1);
        maxNumGoal_editor_->setValue(0);
        maxNumGoal_layout->addWidget(maxNumGoal_editor_);
        cycle_checkbox_ = new QCheckBox("循环");
        maxNumGoal_layout->addWidget(cycle_checkbox_, 0, Qt::AlignRight);
        root_layout->addLayout(maxNumGoal_layout);

        QHBoxLayout *multiNav_layout = new QHBoxLayout;
        singleNavi_button_ = new QPushButton("单点导航");
        multiNav_layout->addWidget(singleNavi_button_);
        startNavi_button_ = new QPushButton("多点导航");
        multiNav_layout->addWidget(startNavi_button_);
        cancel_button_ = new QPushButton("取消导航");
        multiNav_layout->addWidget(cancel_button_);
        root_layout->addLayout(multiNav_layout);

        // creat a QTable to contain the poseArray
        poseArray_table_ = new QTableWidget;
        initPoseTable();
        root_layout->addWidget(poseArray_table_);

        // creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        load_button_ = new QPushButton("加载");
        manipulate_layout->addWidget(load_button_);
        reset_button_ = new QPushButton("重置");
        manipulate_layout->addWidget(reset_button_);
        save_button_ = new QPushButton("保存");
        manipulate_layout->addWidget(save_button_);
        root_layout->addLayout(manipulate_layout);

        control_panel_ = new ControlPanel();
        root_layout->addLayout(control_panel_->layout_);

        setLayout(root_layout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *timer = new QTimer(this);
        timer->start(200); // 200ms, 5hz

        connect(maxNumGoal_editor_, SIGNAL(valueChanged(int)), this, SLOT(updateMaxNumGoal()));
        connect(load_button_, SIGNAL(clicked()), this, SLOT(loadGoals()));
        connect(save_button_, SIGNAL(clicked()), this, SLOT(saveGoals()));
        connect(reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
        connect(startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(singleNavi_button_, SIGNAL(clicked()), this, SLOT(singleNavi()));
        connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(timer, SIGNAL(timeout()), this, SLOT(startSpin()));
        ROS_INFO("AgilexNavigationPanel Initialized!");
    }

    // 更新maxNumGoal命名
    void AgilexNavigationPanel::updateMaxNumGoal()
    {
        maxNumGoal_ = maxNumGoal_editor_->value();
        poseArray_table_->setRowCount(maxNumGoal_);
    }

    // initialize the table of pose
    void AgilexNavigationPanel::initPoseTable()
    {
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        poseArray_table_->clear();
        pose_array_.poses.clear();
        deleteMark();

        poseArray_table_->setRowCount(maxNumGoal_);
        poseArray_table_->setColumnCount(3);
        poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
        /*设置表格为整行选中*/
        poseArray_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        /*设置允许单个选中*/
        poseArray_table_->setSelectionMode(QAbstractItemView::SingleSelection);
        QStringList pose_header;
        pose_header << "x(m)"
                    << "y(m)"
                    << "yaw(度)";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_->setCheckState(Qt::Unchecked);
    }

    void AgilexNavigationPanel::loadGoals()
    {
        QString fileName = QFileDialog::getOpenFileName(this, tr("导入目标点"), QString(), tr("Text files(*.txt)"));
        if (fileName.isEmpty())
            return;

        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        poseArray_table_->clear();
        pose_array_.poses.clear();
        deleteMark();

        QFile file;
        file.setFileName(fileName);
        //打开文件
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            ROS_WARN("Cannot read goals data from file %s", fileName.toStdString().c_str());
            return;
        }
        if (!file.size())
        {
            ROS_WARN("Empty file %s", fileName.toStdString().c_str());
            return;
        }
        //将文件数据导入表格
        int r_count = 0;      //统计文件的行数
        QStringList textList; //记录文件中每一行的数据
        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString line = in.readLine();
            textList.append(line); //保存文件的数据
            r_count++;             //记录文件的行数
        }
        file.close();
        maxNumGoal_ = r_count - 1;
        maxNumGoal_editor_->setValue(maxNumGoal_);

        if (!textList.isEmpty() && r_count > 1)
        {
            QStringList listRowHeader = textList.at(0).split("\t");
            int c_count = listRowHeader.count();
            poseArray_table_->setRowCount(r_count - 1);    //第1行是行表头
            poseArray_table_->setColumnCount(c_count - 1); //最后一列是空
            poseArray_table_->setHorizontalHeaderLabels(listRowHeader);
            pose_array_.header.frame_id = pose_frame_id;

            for (int row = 1; row < r_count; row++)
            {
                QStringList tmpList;
                tmpList = textList.at(row).split("\t");
                geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());
                pose->header.frame_id = pose_frame_id;
                pose->pose.position.x = tmpList.at(0).toDouble();
                pose->pose.position.y = tmpList.at(1).toDouble();
                pose->pose.orientation = tf::createQuaternionMsgFromYaw(tmpList.at(2).toDouble() / 180.0 * M_PI);
                pose_array_.poses.push_back(pose->pose);
                writePose(pose->pose);
                markPose(pose);
            }
            ROS_INFO("Loaded goals data from file %s", fileName.toStdString().c_str());
        }
    }

    void AgilexNavigationPanel::saveGoals()
    {
        QString filepath = QFileDialog::getSaveFileName(this, tr("保存目标点"), QString(), tr("Text files(*.txt)"));

        if (!filepath.isEmpty())
        {
            int row = poseArray_table_->rowCount();
            int col = poseArray_table_->columnCount();

            QList<QString> list;
            //添加列标题
            QString HeaderRow;
            for (int i = 0; i < col; i++)
            {
                HeaderRow.append(poseArray_table_->horizontalHeaderItem(i)->text() + "\t");
            }
            list.push_back(HeaderRow);

            for (int i = 0; i < row; i++)
            {
                QString rowStr = "";
                for (int j = 0; j < col; j++)
                {
                    rowStr += poseArray_table_->item(i, j)->text() + "\t";
                }
                list.push_back(rowStr);
            }
            QTextEdit textEdit;
            for (int i = 0; i < list.size(); i++)
            {
                textEdit.append(list.at(i));
            }
            QFile file(filepath);
            if (file.open(QFile::WriteOnly | QIODevice::Text))
            {
                QTextStream ts(&file);
                ts.setCodec("utf-8");
                ts << textEdit.document()->toPlainText();
                file.close();
            }
        }
    }

    // delete marks in the map
    void AgilexNavigationPanel::deleteMark()
    {
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker_delete);
    }

    // call back function for counting goals
    void AgilexNavigationPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        if (pose_array_.poses.size() < maxNumGoal_)
        {
            pose_array_.poses.push_back(pose->pose);
            pose_array_.header.frame_id = pose->header.frame_id;
            writePose(pose->pose);
            markPose(pose);
        }
        else
        {
            ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
        }
    }

    // write the poses into the table
    void AgilexNavigationPanel::writePose(geometry_msgs::Pose pose)
    {
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,
                                  new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,
                                  new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,
                                  new QTableWidgetItem(
                                      QString::number(tf::getYaw(pose.orientation) * 180.0 / M_PI, 'f', 2)));
    }

    // when setting a Navi Goal, it will set a mark on the map
    void AgilexNavigationPanel::markPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        if (ros::ok())
        {
            visualization_msgs::Marker arrow;
            visualization_msgs::Marker number;
            arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            arrow.action = number.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            arrow.pose = number.pose = pose->pose;
            number.pose.position.z += 1.0;
            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            number.scale.z = 1.0;
            arrow.color.r = number.color.r = 1.0f;
            arrow.color.g = number.color.g = 0.98f;
            arrow.color.b = number.color.b = 0.80f;
            arrow.color.a = number.color.a = 1.0;
            arrow.id = number.id = pose_array_.poses.size();
            number.text = std::to_string(pose_array_.poses.size());
            marker_pub_.publish(arrow);
            marker_pub_.publish(number);
        }
    }

    // check whether it is in the cycling situation
    void AgilexNavigationPanel::checkCycle()
    {
        cycle_ = cycle_checkbox_->isChecked();
    }

    // start to navigate, and only command the first goal
    void AgilexNavigationPanel::startNavi()
    {
        curGoalIdx_ = poseArray_table_->currentRow() < 0 ? 0 : poseArray_table_->currentRow();

        if (!pose_array_.poses.empty())
        {
            if (curGoalIdx_ < maxNumGoal_)
            {
                pubGoal();
            }
            permit_ = true;
        }
        else
        {
            ROS_ERROR("Empty goals!");
        }
    }

    void AgilexNavigationPanel::singleNavi()
    {
        curGoalIdx_ = poseArray_table_->currentRow();
        if (curGoalIdx_ < 0 || curGoalIdx_ > pose_array_.poses.size() - 1)
        {
            ROS_WARN("Invalid Goal %d", curGoalIdx_ + 1);
            return;
        }
        ROS_INFO("Single navi goal %d", curGoalIdx_ + 1);
        permit_ = false;
        pubGoal();
    }

    void AgilexNavigationPanel::pubGoal()
    {
#if USING_NAV_MODE == 0
        geometry_msgs::PoseStamped goal;
        goal.header = pose_array_.header;
        goal.pose = pose_array_.poses.at(curGoalIdx_);
#elif USING_NAV_MODE == 1
        mbf_msgs::MoveBaseActionGoal goal;
        goal.header = pose_array_.header;
        goal.goal.target_pose.header = goal.header;
        goal.goal.target_pose.pose = pose_array_.poses.at(curGoalIdx_);
#else
        locomotor_msgs::NavigateToPoseActionGoal goal;
        goal.header = pose_array_.header;
        goal.goal.goal.header = goal.header;
        geometry_msgs::Pose pose = pose_array_.poses.at(curGoalIdx_);
        goal.goal.goal.pose.x = pose.position.x;
        goal.goal.goal.pose.y = pose.position.y;
        goal.goal.goal.pose.theta = tf::getYaw(pose.orientation);
#endif
        goal_pub_.publish(goal);
        if (!cycle_ || !permit_)
            ROS_INFO("Navi to the Goal %d", curGoalIdx_ + 1);
        else
            ROS_INFO("Navi to the Goal %d, in the %dth cycle", curGoalIdx_ + 1, cycleCnt_ + 1);
        poseArray_table_->selectRow(curGoalIdx_);
    }

    // cancel the current command
    void AgilexNavigationPanel::cancelNavi()
    {
        cancel_pub_.publish(actionlib_msgs::GoalID());
        ROS_INFO("Navigation have been canceled");
        permit_ = false;
    }

    // call back for listening current state
    void AgilexNavigationPanel::movebaseStatusCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &result)
    {
        checkStatus(result->status);
    }

    void AgilexNavigationPanel::movebaseflexStatusCB(const mbf_msgs::MoveBaseActionResult::ConstPtr &result)
    {
        checkStatus(result->status);
        ROS_INFO("mbf navigate result: dist_to_goal=%f, angle_to_goal=%f", result->result.dist_to_goal, result->result.angle_to_goal);
    }

    void AgilexNavigationPanel::locomotorStatusCB(const locomotor_msgs::NavigateToPoseActionResult::ConstPtr &result)
    {
        checkStatus(result->status);
        ROS_INFO("locomotor navigate result: %s", result->result.result_code.message.c_str());
    }

    // check the current state of goal
    void AgilexNavigationPanel::checkStatus(actionlib_msgs::GoalStatus status)
    {
        ROS_INFO("Goal %d status: %d. %s\n", curGoalIdx_ + 1, status.status, status.text.c_str());
        switch (status.status)
        {
        case actionlib_msgs::GoalStatus::PREEMPTED:
        {
            permit_ = false;
            break;
        }
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
        default:
            break;
        }
        if (ros::ok() && permit_)
        {
            curGoalIdx_++;
            curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
            if (cycle_)
            {
                pubGoal();
                cycleCnt_ += (curGoalIdx_ + 1) / pose_array_.poses.size();
            }
            else
            {
                if (curGoalIdx_ > 0)
                {
                    pubGoal();
                }
                else
                {
                    ROS_INFO("All goals are completed");
                    permit_ = false;
                }
            }
        }
    }

    // spin for subscribing
    void AgilexNavigationPanel::startSpin()
    {
        if (ros::ok())
        {
            ros::spinOnce();
        }
    }

} // end namespace agilex_nav_control_rviz_plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agilex_nav_control_rviz_plugin::AgilexNavigationPanel, rviz::Panel)
