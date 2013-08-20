/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

package org.ros.android.shield_teleop;

import android.os.Bundle;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.view.MotionEvent;
import android.view.KeyEvent;
import android.view.InputEvent;
import android.view.InputDevice;
import android.util.Log;

/**
* This class encapsulates the entry point Android Activity for the Shield Teleop app. 
* When utilizing rosjava, it is convenient to subclass the RosActivity (which
* is a subclass of Activity) instead of directly subclassing Activity. This provides
* convenience callbacks to initialize the various nodes that will run within this
* app. Each node is subclassed from NodeMain and runs as a thread instead of as a 
* process as in normal ROS. Each thread is run inside a single service managed by
* the class NodeMainExecutor which allows nodes to keep running in the background
* even if app focus is lost.
*/

public class MainActivity extends RosActivity
{
    private RosImageView<sensor_msgs.CompressedImage> videoStreamView_;            //The rosjava node + widget which renders the video stream
    private JoystickNode                              joystickHandler_;  //The rosjava node which handles joystick events and publishes sensor_msgs/Joy

    /**
    * Constructor for this class
    */
    public MainActivity()
    /***************************************************************************/
    {
        super("ROS SHIELD Teleop", "ROS SHIELD Teleop");
    }

    /** 
    * Triggered when the Activity is created
    */ 
    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState)
    /***************************************************************************/
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        videoStreamView_ = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        videoStreamView_.setTopicName("/usb_cam/videoStreamView_raw/compressed");
        videoStreamView_.setMessageType(sensor_msgs.CompressedImage._TYPE);
        videoStreamView_.setMessageToBitmapCallable(new BitmapFromCompressedImage());

        //twistPublisher_ = new TwistPublisher();

        joystickHandler_ = new JoystickNode();

    }

    /**
    * Until we get a joystick event, it is not certain what InputDevice corressponds to the joystick
    * Though there are methods for introspection, I was in a hurry and this does the job.
    */
    private void initializeJoystickHandlerIfPossibleAndNecessary(InputEvent event)
    /*************************************************************************/
    {
        if ((!joystickHandler_.isInitialized()) && (event.getSource() & InputDevice.SOURCE_CLASS_JOYSTICK) != 0)
            joystickHandler_.initializeDevice(event.getDevice());
    }

    /** 
    * This is an android event handler when key press/release events happen
    */ 
    @Override
    public boolean dispatchKeyEvent(KeyEvent event)
    /*************************************************************************/
    {
        Log.d("MainActivity", "MainActivity: dispatchKeyEvent");
        
        initializeJoystickHandlerIfPossibleAndNecessary(event);

        if (!joystickHandler_.isInitialized())
            return super.dispatchKeyEvent(event);

        switch(event.getAction())
        {
            case KeyEvent.ACTION_DOWN:
                if (joystickHandler_.onKeyDown(event))
                    return true;
                break;
            case KeyEvent.ACTION_UP:
                if (joystickHandler_.onKeyUp(event)) 
                    return true;
                break;   
        }

        return super.dispatchKeyEvent(event);
    }

    /**
    * This is an android event handler for generic motion events. This is triggered
    * when any of the axes controls (vs simple buttons) on the joystick are used
    * where each axis has a value between -1.0 and 1.0.
    */
    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event)
    /*************************************************************************/
    {
        Log.d("MainActivity", "MainActivity: dispatchGenericMotionEvent");

        initializeJoystickHandlerIfPossibleAndNecessary(event);

        boolean isJoystickEvent = ((event.getSource() & InputDevice.SOURCE_CLASS_JOYSTICK) != 0);
        boolean isActionMoveEvent = event.getAction() == MotionEvent.ACTION_MOVE;

        if (!isJoystickEvent || !isActionMoveEvent || !joystickHandler_.isInitialized())
            return super.dispatchGenericMotionEvent(event);
        
        if (joystickHandler_.onJoystickMotion(event))
            return true;

        return super.dispatchGenericMotionEvent(event);
    }

    /**
    * A rosjava callback for RosActivity which gives one the opportunity to spawn
    * nodes (subclasses of NodeMain).
    */
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor)
    /*************************************************************************/
    {
        NodeConfiguration nodeConfiguration1 =
            NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                getMasterUri());
        nodeConfiguration1.setNodeName("ShieldTeleop/ImageView");

        NodeConfiguration nodeConfiguration2 =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                        getMasterUri());
        nodeConfiguration2.setNodeName("ShieldTeleop/JoystickNode");

        //Here we start each node thread (subclass of NodeMain) which run inside an Android service
        //(i.e. basically a persistent background daemon).
        nodeMainExecutor.execute(videoStreamView_, nodeConfiguration1);
        nodeMainExecutor.execute(joystickHandler_, nodeConfiguration2);
    }
}
