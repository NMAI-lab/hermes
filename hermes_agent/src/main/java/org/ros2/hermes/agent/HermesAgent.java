package org.ros2.hermes.agent;

import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.io.FileInputStream;
import java.io.IOException;

import org.yaml.snakeyaml.Yaml;
import org.json.JSONObject;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.concurrent.Callback;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;

import jason.architecture.AgArch;
import jason.asSemantics.Agent;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Literal;
import jason.infra.centralised.*;

public class HermesAgent extends AgArch implements Runnable {
    // ROS parameters
    private final Node node;
    private Subscription<std_msgs.msg.String> beliefSubscription;
    private Subscription<std_msgs.msg.String> actionStatusSubscription;
    private Publisher<std_msgs.msg.String> actionPublisher;

    // Jason related parameters
    private List<ActionExec> pendingActions;
    private boolean actionPerformed;
    private long reasoningTime;

    // General params
    private Map<String, Object> nodeConstants;
    private JSONObject currentPerceptions;

    // Constants
    public static final String ASL_FILE = "jason_agent/hermes_agent.asl";
    public static final String AGENT_PARAMS_FILE = "agent_params.yaml";

    public HermesAgent() {
        this.node = RCLJava.createNode("hermes_agent");
        this.pendingActions = new ArrayList<>();
        this.reasoningTime = 0;
        this.currentPerceptions = null;

        String mapsFile = System.getenv().getOrDefault("map_file", "");
        String agentDefinitionsFolder = System.getenv().getOrDefault("agent_definitions", "");
        String configFolder = System.getenv().getOrDefault("config", "");
        
        // Loading the agent's params
        try (FileInputStream input = new FileInputStream(configFolder + "/" + AGENT_PARAMS_FILE)) {
            Yaml yaml = new Yaml();
            this.nodeConstants = yaml.load(input);
        } catch (IOException e) {
            System.out.println("COULD NOT LOAD AGENT PARAMETERS");
            e.printStackTrace();
            return;
        }

        // Node publishers
        this.actionPublisher = this.node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, (String)this.nodeConstants.get("actions_publisher_topic"));

        // Node subscribers
        this.beliefSubscription = this.node.<std_msgs.msg.String>createSubscription(std_msgs.msg.String.class, (String)this.nodeConstants.get("beliefs_subscriber_topic"),
                                                                                    this::beliefCallback);
        this.actionStatusSubscription = this.node.<std_msgs.msg.String>createSubscription(std_msgs.msg.String.class,
                                                                                          (String)this.nodeConstants.get("action_status_subscriber_topic"), this::actionStatusCallback);

        // set up the Jason agent
        try {
            Agent ag = new Agent();
            new TransitionSystem(ag, null, null, this);
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
                long startTime = System.currentTimeMillis();
                getTS().reasoningCycle();
                long endTime = System.currentTimeMillis();
                reasoningTime = endTime - startTime;
                getTS().getLogger().info("Reasoning Time: " + Long.toString(reasoningTime));

                if (getTS().canSleep()) {
                    getTS().getLogger().info("Agent sleeping...");
                    actionPerformed = false;
                    sleep();
                } else {
                    getTS().getLogger().info("Agent cannot sleep...");
                }
            }
        } catch (Exception e) {
            getTS().getLogger().log(Level.SEVERE, "Run error", e);
        }
    }

    @Override
    public List<Literal> perceive() {
        List<Literal> l = new ArrayList<Literal>();
        getTS().getLogger().info("Agent " + getAgName() + " is perceiving...");

        // Generating beliefs from the perceptions
        if (this.currentPerceptions != null) {
            if (this.currentPerceptions.has("right_wall_dist") && this.currentPerceptions.has("right_wall_angle")) {
                l.add(Literal.parseLiteral("facing_wall(" + Double.toString(this.currentPerceptions.getDouble("right_wall_dist")) + "," +  Double.toString(this.currentPerceptions.getDouble("right_wall_angle")) + ")"));
            }
            this.currentPerceptions = null;
        }

        // Loading the constants
        Double wall_follow_distance_setpoint = (Double)this.nodeConstants.get("wall_follow_distance_setpoint");
        Double wall_follow_aim_angle = (Double)this.nodeConstants.get("wall_follow_aim_angle");
        Double wall_follow_speed = (Double)this.nodeConstants.get("wall_follow_speed");
        Double wall_follow_angle_change_tolerance = (Double)this.nodeConstants.get("wall_follow_angle_change_tolerance");
        Double error = wall_follow_speed * Math.sin(wall_follow_aim_angle * Math.PI / 180.0);

        l.add(Literal.parseLiteral("wall_follow_distance_setpoint(" + Double.toString(wall_follow_distance_setpoint) + ")"));
        l.add(Literal.parseLiteral("wall_follow_aim_angle(" + Double.toString(wall_follow_aim_angle) + ")"));
        l.add(Literal.parseLiteral("wall_follow_angle_change_tolerance(" + Double.toString(wall_follow_angle_change_tolerance) + ")"));
        l.add(Literal.parseLiteral("wall_follow_speed(" + Double.toString(wall_follow_speed) + ")"));
        l.add(Literal.parseLiteral("wall_follow_distance_tolerance(" + Double.toString(error) + ")"));
        
        getTS().getLogger().info("Agent " + getAgName() + " beliefs are: " + l);

        return l;
    }

    // this method get the agent actions
    @Override
    public void act(ActionExec action) {
        getTS().getLogger().info("Agent " + getAgName() + " is doing: " + action.getActionTerm());
        pendingActions.add(action);

        std_msgs.msg.String message = new std_msgs.msg.String();
        JSONObject msgContent = new JSONObject();
        msgContent.put("name", action.getActionTerm().toString());
        msgContent.put("action_id", action.hashCode());
        message.setData(msgContent.toString());
        this.actionPublisher.publish(message);
    }

    @Override
    public boolean canSleep() {
        return actionPerformed;
    }

    @Override
    public boolean isRunning() {
        return true;
    }

    public void sleep() {
        try {
            Thread.sleep(1000 - reasoningTime);
        } catch (InterruptedException e) {}
    }

    public String getAgName() {
        return "hermes";
    }

    private void beliefCallback(final std_msgs.msg.String msg){
        this.currentPerceptions = new JSONObject(msg.getData());
    }

    private void actionStatusCallback(final std_msgs.msg.String msg){
        JSONObject obj = new JSONObject(msg.getData());
        
        int executedActionHash = obj.getInt("action_id");
        ActionExec executedAction = null;

        for (ActionExec action: pendingActions) {
            if (action.hashCode() == executedActionHash){
                executedAction = action;
                break;
            }
        }

        // set that the execution was ok
        actionPerformed = true;
        executedAction.setResult(true);
        actionExecuted(executedAction);
        pendingActions.remove(executedAction);
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