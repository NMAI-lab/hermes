package org.ros2.hermes.agent;

import java.util.concurrent.TimeUnit;
import org.json.JSONObject;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.concurrent.Callback;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.timer.WallTimer;

public class HermesAgent {
    // ROS parameters
    private final Node node;
    private Subscription<std_msgs.msg.String> belief_subscription;
    private Publisher<std_msgs.msg.String> action_publisher;

    private WallTimer timer;
    private String lastBeliefs;

    public HermesAgent() {
        this.node = RCLJava.createNode("hermes_agent");
        this.lastBeliefs = "";

        String agentParams = System.getenv().getOrDefault("map_file", "map.json");
        System.out.println("AGENT_PARAMS env var: " + agentParams);
        
        // Publishers are type safe, make sure to pass the message type
        this.belief_subscription = this.node.<std_msgs.msg.String>createSubscription(std_msgs.msg.String.class, "/beliefs", this::beliefCallback);
        this.action_publisher = this.node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "/actions");

        /*Callback timerCallback = () -> {
            std_msgs.msg.String message = new std_msgs.msg.String();
            message.setData("Hello, world! " + this.count);
            this.count++;
            System.out.println("Publishing: [" + message.getData() + "]");
            this.publisher.publish(message);
        };
        this.timer = this.node.createWallTimer(500, TimeUnit.MILLISECONDS, timerCallback);*/
    }

    private void beliefCallback(final std_msgs.msg.String msg){
        JSONObject obj = new JSONObject(msg.getData());
        System.out.println("I heard: " + obj.toString());
    }

    public Node getNode() {
        return this.node;
    }

    public static void main(String[] args) throws InterruptedException {
        // Initialize RCL
        RCLJava.rclJavaInit();

        HermesAgent agent = new HermesAgent();
        RCLJava.spin(agent.getNode());
    }
}