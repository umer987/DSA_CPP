package singleFileTrafficSimulation;

import java.util.*;
import java.util.concurrent.*;

public class SmartCityTrafficSimulation {
    
    // Enums
    enum WeatherCondition {
        NORMAL(1.0), RAIN(0.7), FOG(0.5), SNOW(0.3);
        
        private final double speedFactor;
        
        WeatherCondition(double speedFactor) {
            this.speedFactor = speedFactor;
        }
        
        public double getSpeedFactor() {
            return speedFactor;
        }
        
        public static WeatherCondition randomWeather() {
            WeatherCondition[] values = values();
            return values[(int)(Math.random() * values.length)];
        }
    }
    
    enum TrafficLightState {
        GREEN_NS, YELLOW_NS, GREEN_EW, YELLOW_EW;
        
        public TrafficLightState next() {
            return switch (this) {
                case GREEN_NS -> YELLOW_NS;
                case YELLOW_NS -> GREEN_EW;
                case GREEN_EW -> YELLOW_EW;
                case YELLOW_EW -> GREEN_NS;
            };
        }
    }
    
    // Classes
    static class Intersection {
        private final int id;
        
        public Intersection(int id) {
            this.id = id;
        }
        
        public int getId() { return id; }
    }
    
    static class Road {
        private final int id;
        private final int from;
        private final int to;
        private final int length;
        private final int speedLimit;
        private int vehicleCount;
        private double currentSpeed;
        
        public Road(int id, int from, int to, int length, int speedLimit) {
            this.id = id;
            this.from = from;
            this.to = to;
            this.length = length;
            this.speedLimit = speedLimit;
            this.vehicleCount = 0;
            this.currentSpeed = speedLimit;
        }
        
        public void updateLoad() {
            double congestionFactor = 1.0 - (0.05 * vehicleCount);
            currentSpeed = Math.max(5, speedLimit * congestionFactor);
        }
        
        public void vehicleEntered() {
            vehicleCount++;
            updateLoad();
        }
        
        public void vehicleExited() {
            vehicleCount--;
            updateLoad();
        }
        
        // Getters
        public int getId() { return id; }
        public int getFrom() { return from; }
        public int getTo() { return to; }
        public int getLength() { return length; }
        public int getSpeedLimit() { return speedLimit; }
        public int getVehicleCount() { return vehicleCount; }
        public double getCurrentSpeed() { return currentSpeed; }
    }
    
    static class Route {
        private final List<Integer> path;
        private final double estimatedTime;
        
        public Route(List<Integer> path, double estimatedTime) {
            this.path = path;
            this.estimatedTime = estimatedTime;
        }
        
        public List<Integer> getPath() { return path; }
        public double getEstimatedTime() { return estimatedTime; }
    }
    
    static class Vehicle {
        private final int id;
        private final int destination;
        private int currentLocation;
        private Route currentRoute;
        private final boolean isEmergencyVehicle;
        private int routeProgress;
        private double speed;
        
        public Vehicle(int id, int start, int destination, boolean isEmergencyVehicle) {
            this.id = id;
            this.currentLocation = start;
            this.destination = destination;
            this.isEmergencyVehicle = isEmergencyVehicle;
            this.routeProgress = 0;
            this.speed = isEmergencyVehicle ? 1.5 : 1.0;
        }
        
        public void move() {
            if (hasArrived()) return;
            
            routeProgress++;
            if (routeProgress < currentRoute.getPath().size()) {
                currentLocation = currentRoute.getPath().get(routeProgress);
            }
        }
        
        public boolean hasArrived() {
            return currentLocation == destination;
        }
        
        public void updateRoute(Route newRoute) {
            this.currentRoute = newRoute;
            this.routeProgress = 0;
        }
        
        // Getters
        public int getId() { return id; }
        public int getCurrentLocation() { return currentLocation; }
        public int getDestination() { return destination; }
        public Route getCurrentRoute() { return currentRoute; }
        public boolean isEmergencyVehicle() { return isEmergencyVehicle; }
        public void setSpeed(double speed) { this.speed = speed; }
    }
    
    static class TrafficLightController {
        private final CityGraph city;
        private final Map<Integer, TrafficLightState> intersectionStates;
        private final Map<Integer, Integer> lightTimers;
        private Set<Integer> emergencyPath;
        private boolean emergencyMode;
        
        public TrafficLightController(CityGraph city) {
            this.city = city;
            this.intersectionStates = new HashMap<>();
            this.lightTimers = new HashMap<>();
            this.emergencyPath = new HashSet<>();
            this.emergencyMode = false;
            
            initializeTrafficLights();
        }
        
        private void initializeTrafficLights() {
            for (Integer intersectionId : city.getIntersections().keySet()) {
                intersectionStates.put(intersectionId, TrafficLightState.GREEN_NS);
                lightTimers.put(intersectionId, 0);
            }
        }
        
        public void updateTrafficLights() {
            for (Integer intersectionId : intersectionStates.keySet()) {
                int timer = lightTimers.get(intersectionId) + 1;
                
                if (timer > getLightDuration(intersectionId)) {
                    rotateLightState(intersectionId);
                    timer = 0;
                }
                
                lightTimers.put(intersectionId, timer);
            }
        }
        
        private int getLightDuration(int intersectionId) {
            if (emergencyPath.contains(intersectionId)) {
                return 2;
            }
            return 5 + (int)(Math.random() * 3);
        }
        
        private void rotateLightState(int intersectionId) {
            TrafficLightState current = intersectionStates.get(intersectionId);
            TrafficLightState next = current.next();
            intersectionStates.put(intersectionId, next);
        }
        
        public boolean canMove(Vehicle vehicle, Intersection intersection) {
            if (vehicle.isEmergencyVehicle() && emergencyMode) {
                return true;
            }
            
            TrafficLightState state = intersectionStates.get(intersection.getId());
            return state == TrafficLightState.GREEN_NS || state == TrafficLightState.GREEN_EW;
        }
        
        public void setEmergencyPath(Route route) {
            this.emergencyPath = new HashSet<>(route.getPath());
            this.emergencyMode = true;
        }
        
        public void clearEmergency() {
            this.emergencyPath.clear();
            this.emergencyMode = false;
        }
    }
    
    static class CityGraph {
        private final Map<Integer, Intersection> intersections;
        private final Map<Integer, Road> roads;
        private final Map<Integer, List<Road>> adjacencyList;
        
        public CityGraph(int intersectionCount, int roadCount) {
            this.intersections = new HashMap<>();
            this.roads = new HashMap<>();
            this.adjacencyList = new HashMap<>();
            
            initializeCity(intersectionCount, roadCount);
        }
        
        private void initializeCity(int intersectionCount, int roadCount) {
            // Create intersections
            for (int i = 0; i < intersectionCount; i++) {
                intersections.put(i, new Intersection(i));
            }
            
            // Connect intersections with roads
            Random rand = new Random();
            for (int i = 0; i < roadCount; i++) {
                int from = rand.nextInt(intersectionCount);
                int to = rand.nextInt(intersectionCount);
                while (from == to) {
                    to = rand.nextInt(intersectionCount);
                }
                
                int length = 1 + rand.nextInt(10);
                int speedLimit = 30 + rand.nextInt(50);
                
                Road road = new Road(i, from, to, length, speedLimit);
                roads.put(i, road);
                
                adjacencyList.computeIfAbsent(from, k -> new ArrayList<>()).add(road);
                // For undirected graph (two-way roads)
                adjacencyList.computeIfAbsent(to, k -> new ArrayList<>()).add(
                    new Road(i + roadCount, to, from, length, speedLimit)
                );
            }
        }
        
        public Optional<Route> findShortestPath(int start, int destination, WeatherCondition weather) {
            Map<Integer, Double> distances = new HashMap<>();
            Map<Integer, Integer> previous = new HashMap<>();
            PriorityQueue<IntersectionDistance> pq = new PriorityQueue<>();
            
            for (Integer intersectionId : intersections.keySet()) {
                distances.put(intersectionId, Double.POSITIVE_INFINITY);
            }
            distances.put(start, 0.0);
            pq.offer(new IntersectionDistance(start, 0));
            
            while (!pq.isEmpty()) {
                IntersectionDistance current = pq.poll();
                int currentId = current.intersectionId;
                
                if (currentId == destination) break;
                
                for (Road road : adjacencyList.getOrDefault(currentId, Collections.emptyList())) {
                    double travelTime = calculateTravelTime(road, weather);
                    double newDistance = distances.get(currentId) + travelTime;
                    
                    if (newDistance < distances.get(road.getTo())) {
                        distances.put(road.getTo(), newDistance);
                        previous.put(road.getTo(), currentId);
                        pq.offer(new IntersectionDistance(road.getTo(), newDistance));
                    }
                }
            }
            
            if (distances.get(destination) == Double.POSITIVE_INFINITY) {
                return Optional.empty();
            }
            
            List<Integer> path = new LinkedList<>();
            for (Integer at = destination; at != null; at = previous.get(at)) {
                path.add(0, at);
            }
            
            return Optional.of(new Route(path, distances.get(destination)));
        }
        
        private double calculateTravelTime(Road road, WeatherCondition weather) {
            double baseTime = road.getLength() / road.getCurrentSpeed();
            return baseTime * weather.getSpeedFactor();
        }
        
        public void updateRoadLoads() {
            roads.values().forEach(Road::updateLoad);
        }
        
        public void addVehicle(Vehicle vehicle) {
            // Implementation would update road loads
        }
        
        public void updateVehiclePosition(Vehicle vehicle) {
            // Implementation would update road loads
        }
        
        public Intersection getIntersection(int id) {
            return intersections.get(id);
        }
        
        public Map<Integer, Intersection> getIntersections() {
            return intersections;
        }
        
        public void printTrafficStatus() {
            System.out.println("\nCity Traffic Status:");
            System.out.printf("%-15s %-15s %-15s %-15s%n", "Road ID", "From-To", "Vehicles", "Avg Speed");
            System.out.printf("        " +"PATH-FINDER");
            System.out.println();
            roads.values().forEach(road -> {
                System.out.printf("%-15d %-5d -> %-5d %-15d %-15.1f%n",
                    road.getId(),
                    road.getFrom(),
                    road.getTo(),
                    road.getVehicleCount(),
                    road.getCurrentSpeed());
            });
        }
        
        private static class IntersectionDistance implements Comparable<IntersectionDistance> {
            final int intersectionId;
            final double distance;
            
            IntersectionDistance(int intersectionId, double distance) {
                this.intersectionId = intersectionId;
                this.distance = distance;
            }
            
            @Override
            public int compareTo(IntersectionDistance other) {
                return Double.compare(this.distance, other.distance);
            }
        }
    }
    
    static class TrafficManagementSystem {
        private final CityGraph city;
        private final List<Vehicle> vehicles;
        private final ExecutorService simulationExecutor;
        private final TrafficLightController trafficLightController;
        private WeatherCondition currentWeather;
        private int simulationTime;
        
        public TrafficManagementSystem(int intersections, int roads) {
            this.city = new CityGraph(intersections, roads);
            this.vehicles = new ArrayList<>();
            this.simulationExecutor = Executors.newFixedThreadPool(4);
            this.trafficLightController = new TrafficLightController(city);
            this.currentWeather = WeatherCondition.NORMAL;
            this.simulationTime = 0;
        }
        
        public void runSimulation(int steps) {
            for (int i = 0; i < steps; i++) {
                simulationTime++;
                updateTrafficConditions();
                moveVehicles();
                trafficLightController.updateTrafficLights();
                printSimulationStatus();
                
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        
        private void updateTrafficConditions() {
            city.updateRoadLoads();
            
            if (simulationTime % 20 == 0) {
                currentWeather = WeatherCondition.randomWeather();
            }
        }
        
        private void moveVehicles() {
            for (Vehicle vehicle : vehicles) {
                if (vehicle.hasArrived()) continue;
                
                Intersection currentIntersection = city.getIntersection(vehicle.getCurrentLocation());
                if (trafficLightController.canMove(vehicle, currentIntersection)) {
                    vehicle.move();
                    
                    if (simulationTime % 5 == 0) {
                        Optional<Route> newRoute = calculateOptimalRoute(vehicle);
                        newRoute.ifPresent(vehicle::updateRoute);
                    }
                }
            }
        }
        
        private Optional<Route> calculateOptimalRoute(Vehicle vehicle) {
            return city.findShortestPath(
                vehicle.getCurrentLocation(),
                vehicle.getDestination(),
                currentWeather
            );
        }
        
        public void addVehicle(Vehicle vehicle) {
            Route initialRoute = calculateOptimalRoute(vehicle)
                .orElseThrow(() -> new IllegalArgumentException("No valid route found"));
            vehicle.updateRoute(initialRoute);
            vehicles.add(vehicle);
        }
        
        public void declareEmergency(Vehicle emergencyVehicle) {
            trafficLightController.setEmergencyPath(emergencyVehicle.getCurrentRoute());
        }
        
        public void endEmergency() {
            trafficLightController.clearEmergency();
        }
        
        private void printSimulationStatus() {
            System.out.println("\n=== Simulation Time: " + simulationTime + " ===");
            System.out.println("Weather: " + currentWeather);
            System.out.println("Active vehicles: " + vehicles.stream().filter(v -> !v.hasArrived()).count());
            city.printTrafficStatus();
        }
        
        public void shutdown() {
            simulationExecutor.shutdownNow();
        }
    }
    
    // Main method to run the simulation
    public static void main(String[] args) {
        TrafficManagementSystem tms = new TrafficManagementSystem(10, 20);
        
        // Add regular vehicles
        for (int i = 0; i < 15; i++) {
            int start = (int)(Math.random() * 10);
            int end = (int)(Math.random() * 10);
            while (end == start) {
                end = (int)(Math.random() * 10);
            }
            tms.addVehicle(new Vehicle(i, start, end, false));
        }
        
        // Add an emergency vehicle
        Vehicle ambulance = new Vehicle(99, 0, 9, true);
        tms.addVehicle(ambulance);
        // Run simulation
        tms.runSimulation(10);
        
        // Declare emergency
        tms.declareEmergency(ambulance);
        tms.runSimulation(5);
        
        // End emergency
        tms.endEmergency();
        tms.runSimulation(5);
        
        tms.shutdown();
    }
}