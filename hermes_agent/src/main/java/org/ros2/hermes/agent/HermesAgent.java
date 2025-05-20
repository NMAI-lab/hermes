package org.ros2.hermes.agent;

import java.util.concurrent.TimeUnit;
import java.util.List;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.json.JSONObject;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.concurrent.Callback;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.timer.WallTimer;

import jason.architecture.AgArch;
import jason.asSemantics.Agent;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Literal;
import jason.infra.centralised.*;

public class HermesAgent extends AgArch implements Runnable {
    // ROS parameters
    private final Node node;
    private Subscription<std_msgs.msg.String> belief_subscription;
    private Publisher<std_msgs.msg.String> action_publisher;

    private WallTimer timer;
    private String lastBeliefs;

    private boolean actionPerformed;

    private long reasoningTime;

    public static final String ASL_FILE = "jason_agent/hermes_agent.asl";
    public static final String LOGGING_FILE = "jason_agent/logging.properties";

    public HermesAgent() {
        this.node = RCLJava.createNode("hermes_agent");
        this.lastBeliefs = "";

        String mapsFile = System.getenv().getOrDefault("map_file", "");
        String agentDefinitionsFolder = System.getenv().getOrDefault("agent_definitions", "");
        String configFolder = System.getenv().getOrDefault("config", "");
        
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

        //RunCentralisedMAS.main(new String[]{"path/to/your/file.mas2j"});

        // set up the Jason agent

        try {
            Agent ag = new Agent();
            new TransitionSystem(ag, null, null, this);
            String currentPath = System.getProperty("user.dir");
            ag.initAg(agentDefinitionsFolder + "/" + ASL_FILE);
        } catch (Exception e) {
            getTS().getLogger().log(Level.SEVERE, "Could not setup the agent!", e);
        }
    }

    public void run() {
        try {
            while (isRunning()) {
                // calls the Jason engine to perform one reasoning cycle
                getTS().getLogger().info("Reasoning....");
                getTS().reasoningCycle();
                if (getTS().canSleep())
                    sleep();
            }
        } catch (Exception e) {
            getTS().getLogger().log(Level.SEVERE, "Run error", e);
        }
    }

    @Override
    public List<Literal> perceive() {
        getTS().getLogger().info("Agent " + getAgName() + " is perceiving...");
        List<Literal> l = new ArrayList<Literal>();
        l.add(Literal.parseLiteral("x(10)"));
        return l;
    }

    // this method get the agent actions
    @Override
    public void act(ActionExec action) {
        getTS().getLogger().info("Agent " + getAgName() + " is doing: " + action.getActionTerm());

        // set that the execution was ok
        actionPerformed = true;
        action.setResult(true);
        actionExecuted(action);
    }

    @Override
    public boolean canSleep() {
        return true;
    }

    @Override
    public boolean isRunning() {
        return true;
    }

    // a very simple implementation of sleep
    public void sleep() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {}
    }

    public String getAgName() {
        return "hermes";
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
        new RunCentralisedMAS().setupLogger();
        HermesAgent agent = new HermesAgent();
        new Thread(agent).start();
        RCLJava.spin(agent.getNode());
    }
}