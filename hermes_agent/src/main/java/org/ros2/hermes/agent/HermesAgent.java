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
    // A list of pending Jason's actions. These actions are pending the `executed` confirmation.
    private List<ActionExec> pendingActions;
    // Whether an action was performed or not.
    private boolean actionPerformed;
    // Jason's reasoning time.
    private long reasoningTime;

    // General params
    private Map<String, Object> nodeConstants;
    // The current perceptions of the agent.
    private JSONObject currentPerceptions;

    // Constants
    public static final String ASL_FILE = "jason_agent/hermes_agent.asl";
    public static final String AGENT_PARAMS_FILE = "agent_params.yaml";

    /**
     * The main constructor for the agent.
     * Sets up all the ROS connections and starts a JASON agent.
     */
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

    /**
     * The run function for the JASON thread; in charge of communicating with JASON.
     */
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

    // JASON METHODS

    /**
     * The callback for JASON's beliefs retrieval function.
     * Converts the current beliefs of the system to JASON interpretable literals.
     * 
     * @return the beliefs as a list of literals.
     */
    @Override
    public List<Literal> perceive() {
        List<Literal> l = new ArrayList<Literal>();
        getTS().getLogger().info("Agent " + getAgName() + " is perceiving...");

        // Generating beliefs from the perceptions
        JSONObject currentPerceptions = getPerceptions();
        if (currentPerceptions != null) {
            if (currentPerceptions.has("right_wall_dist") && currentPerceptions.has("right_wall_angle")) {
                l.add(Literal.parseLiteral("facing_wall(" + Double.toString(currentPerceptions.getDouble("right_wall_dist")) + "," +  Double.toString(currentPerceptions.getDouble("right_wall_angle")) + ")"));
            }
            resetPerceptions();
        }

        // Loading the constants
        Double speed = (Double)this.nodeConstants.get("speed");
        Double angle_change_tolerance = (Double)this.nodeConstants.get("angle_change_tolerance");
        Double wall_follow_distance_setpoint = (Double)this.nodeConstants.get("wall_follow_distance_setpoint");
        Double wall_follow_aim_angle = (Double)this.nodeConstants.get("wall_follow_aim_angle");

        l.add(Literal.parseLiteral("speed(" + Double.toString(speed) + ")"));
        l.add(Literal.parseLiteral("angle_change_tolerance(" + Double.toString(angle_change_tolerance) + ")"));
        l.add(Literal.parseLiteral("wall_follow_distance_setpoint(" + Double.toString(wall_follow_distance_setpoint) + ")"));
        l.add(Literal.parseLiteral("wall_follow_aim_angle(" + Double.toString(wall_follow_aim_angle) + ")"));
        
        getTS().getLogger().info("Agent " + getAgName() + " beliefs are: " + l);

        return l;
    }

    /**
     * The callback for when JASON has published an action.
     * Publishes the given action over ROS.
     */
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

    /**
     * The method in charge of deciding whether the JASON agent can sleep or not.
     * The agent will only sleep after performing an action.
     */
    @Override
    public boolean canSleep() {
        return actionPerformed;
    }

    /**
     * The method indicating that the agent is running.
     */
    @Override
    public boolean isRunning() {
        return true;
    }

    /**
     * The method to handle that agent's sleep.
     */
    public void sleep() {
        try {
            Thread.sleep(1000 - reasoningTime);
        } catch (InterruptedException e) {}
    }

    /**
     * Returns the name of the agent.
     * 
     * @return the agent name in string format.
     */
    public String getAgName() {
        return "hermes";
    }

    // ROS METHODS

    /**
     * The callback for ROS' /beliefs subscription.
     * Stores the beliefs it just received.
     * 
     * @param msg: the belief that was received through ROS 
     */
    private void beliefCallback(final std_msgs.msg.String msg){
        updatePerceptions(new JSONObject(msg.getData()));
    }

    /**
     * The call for ROS' /action_status subscription.
     * Verifies that the action was received by the `action_translator` node
     * and removes it from the list of pending actions.
     * 
     * @param msg: the action status received through ROS.
     */
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

    /**
     * Obtains the ROS node for the class.
     * 
     * @return the ros node.
     */
    public Node getNode() {
        return this.node;
    }


    // Internal Methods

    /**
     * A synchronized method for updating the current perceptions.
     * 
     * @param newPercepts: new perceptions to add to the class.
     */
    private synchronized void updatePerceptions(JSONObject newPercepts) {
        this.currentPerceptions = newPercepts;
    }

    /**
     * A synchronized method for resetting the current perceptions.
     * Sets the perceptions as null.
     */
    private synchronized void resetPerceptions() {
        this.currentPerceptions = null;
    }

    /**
     * A synchronized method for obtaining the current perceptions.
     * 
     * @return the current perceptions.
     */
    private synchronized JSONObject getPerceptions(){
        return this.currentPerceptions;
    }

    /**
     * The entry point for the process.
     * Initializes the ROS node and spins up the agent thread.
     */
    public static void main(String[] args) throws InterruptedException {
        // Initialize RCL
        RCLJava.rclJavaInit();
        new RunCentralisedMAS().setupLogger();
        HermesAgent agent = new HermesAgent();
        new Thread(agent).start();
        RCLJava.spin(agent.getNode());
    }
}