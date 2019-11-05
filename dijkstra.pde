import java.util.Queue; //<>//
import java.util.LinkedList;


final int GRIDWT = 192;
final int GRIDHT = 108;
final int MASTER_OFFSET_X = 0;
final int MASTER_OFFSET_Y = 50;
final boolean STEPBYSTEP = false;
Dijkstra grid;

void settings(){
  size(1920+MASTER_OFFSET_X, 1080+MASTER_OFFSET_Y);
}
void setup() {
  colorMode(RGB, 1.0);
  frameRate(1000);
  background(0);
  grid = new Dijkstra(GRIDWT, GRIDHT, STEPBYSTEP);
}

void draw() {
  
  grid.tickCommandQueue();
  
  if(grid.isDone()){
    grid.tracePath();
  }
  
  for (Node node : grid.getNodes()) {
    if(null != node){
      node.render();
    }
  }
  
  // FPS counter
  fill(0);
  rect(0,0,width, MASTER_OFFSET_Y);
  fill(0.98,0.85,0.13);
  text("FPS: "+frameRate, 0, MASTER_OFFSET_Y-30);
  
  // Helper text
  fill(1.0, 0.3, 0.3);
  text("Left click: Destination", 100, MASTER_OFFSET_Y-30);
  fill(0.5, 1.0, 0.5);
  text("Right click: Source", 100, MASTER_OFFSET_Y-10);
}

void mousePressed() {
  int coordX = int((mouseX - MASTER_OFFSET_X) / Node.SIZE);
  int coordY = int((mouseY - MASTER_OFFSET_Y) / Node.SIZE);
  Node node = grid.getNode(coordX, coordY);
  if(null != node){ 
    if (mouseButton == LEFT) {
      Node old = grid.setDestination(node);
  
      // Reset old colors.
      if (null != old) {
        old.setColor(0.5, 0.5, 0.5, 1.0);
      }
  
      // Set new colors.
      node.setColor(1.0, 0.3, 0.3, 1.0);
      
    } else if ( mouseButton == RIGHT) {
      Node old = grid.setSource(node);
  
      // Reset old colors.
      if (null != old) {
        old.setColor(0.5, 0.5, 0.5, 1.0);
      }
  
      // Set new colors.
      node.setColor(0.5, 1.0, 0.5, 1.0);
    }
    
    // Can do pathfinding.
    if (null != grid.getDestination() && null != grid.getSource()){
      grid.pathfind();
    }
  }

  
  
}


class Dijkstra {
  private static final float INFINITY = MAX_FLOAT; 
  private Node _destination;
  private Node _source;
  private Node[] _nodes;
  private int _gridWt;
  private int _gridHt;
  private Queue<NodeCommand> _commandQueue;
  private boolean _stepByStep;
  private boolean _isDone;

  public Dijkstra(int gridWt, int gridHt, boolean stepByStep) {
    this._gridWt = gridWt;
    this._gridHt = gridHt;
    this._stepByStep = stepByStep;
    this._isDone = false;
    this._commandQueue = new LinkedList();
    
    _nodes = new Node[GRIDHT*GRIDWT];
    NodeLinkManager nodeLinkManager = new NodeLinkManager();

    // Set nodes 
    for (int y = 0; y < this._gridHt; y++) {
      for (int x = 0; x < this._gridWt; x++) {
        setNode(x, y, new Node(x, y));
      }
    }

    // Set up links
    for (int y = 0; y < this._gridHt; y++) {
      for (int x = 0; x < this._gridWt; x++) {
        if (x > 0) {
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x-1, y), 1.0);
        }
        if (x < this._gridWt-1) {
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x+1, y), 1.0);
        }
        if (y > 0) {
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x, y-1), 1.0);
        }
        if (y < this._gridHt-1) {
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x, y+1), 1.0);
        }
        
        if(x > 0 && y > 0){
           nodeLinkManager.createForNodes(getNode(x, y), getNode(x-1, y-1), 1.0);
        }
        if(x < this._gridWt-1 && y < this._gridHt-1){
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x+1, y+1), 1.0);
        }
        if(x > 0 && y < this._gridHt-1){
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x-1, y+1), 1.0);
        }
        if(x < this._gridWt-1 && y > 0){
          nodeLinkManager.createForNodes(getNode(x, y), getNode(x+1, y-1), 1.0);
        }
       
      }
    }
  }

  // @brief Resets all the nodes' distances to 0.0.
  //        All but the source and destination nodes' color will be set to default.
  public void zeroDistances() {
    for (int y = 0; y < this._gridHt; y++) {
      for (int x = 0; x < this._gridWt; x++) {
        Node node = getNode(x, y);
        node.distance = 0.0;
        if(node != this.getSource() && node != this.getDestination()){
          node.setColor(0.5, 0.5, 0.5, 1.0);
        }
      }
    }
  }
  
  // @brief Sets all node to unvisited.
  public void unvisitAll(){
    for (int y = 0; y < this._gridHt; y++) {
      for (int x = 0; x < this._gridWt; x++) {
        getNode(x, y).setVisited(false);
      }
    }
  }
  
  // @brief Sets all nodes' distances but the source node's to Dijkstra.INFINITY. 
  //        The source node's distance is set to 0.0.
  //        All nodes will be set to unvisited.
  //        All but the source and destination nodes' color will be set to default.
  public void prepareDistances(){
    for (int y = 0; y < this._gridHt; y++) {
      for (int x = 0; x < this._gridWt; x++) {
        Node n = getNode(x, y); 
        n.distance = Dijkstra.INFINITY;
        
        if(this.getDestination() == n){
          n.distance = 0.0;
        }
        
        n.setVisited(false);
        if(n != this.getSource() && n != this.getDestination()){
          n.setColor(0.5, 0.5, 0.5, 1.0);
        }
      }
    }
    
  }
  
  // @brief Runs a single command from the queue.
  public void tickCommandQueue(){
    if(!this._commandQueue.isEmpty()){
      if(this._stepByStep){
        this._commandQueue.poll().run();
      }
    }else{
      if(!this._isDone && this._stepByStep && null != this.getDestination() && null != this.getSource()){
        this._isDone = true;
        System.out.println("done!");
      }
    }
    
  }
  
  // @brief Sets up, runs and sets the colors for the pathfinding.
  public void pathfind(){
    
    this._isDone = false;
    this._commandQueue.clear();
    this.prepareDistances();
    Node starting = this.getDestination();
   
    System.out.println("starting");
    this._commandQueue.add(new NodeTentativeCommand(this, starting));
    
    if(!this._stepByStep){
      while(!this._commandQueue.isEmpty()){
        this._commandQueue.poll().run();
      }
      
      this._isDone = true;
      System.out.println("done!");
    }
    
  }
  
  // @brief Visits a node. Creates necessary neighbours tentative commands.
  public void visit(Node current){
      if(null != current && !current.getVisited()){
        current.setVisited(true);  //<>//
        
        if(this._stepByStep && current != this.getDestination() && current != this.getSource()){
          current.setColor(0.0, 0.0, 0.25 + (float)Math.random() * 0.75, 1.0 );
        }
        
        // Tentate neighbours.
        for (NodeLink nl : current.links) {
          if (!nl.nodeB.getVisited()) {
            this._commandQueue.add(new NodeTentativeCommand(this, nl.nodeB));
          }
        }
      }
  }
  
  // @brief Calculates the minimum distance of neighbouring nodes, then visits the current node.
  public void tentative(Node current){
    if(null != current && !current.getVisited()){
      
      if(this._stepByStep && current != this.getDestination() && current != this.getSource()){
          current.setColor(0.25 + (float)Math.random() * 0.75, 0.0, 0.0, 1.0 );
        }
      
      // Calc neighbours distance.
      for (NodeLink nl : current.links) {
        final float newDist = current.distance + nl.weight;
        if (nl.nodeB.distance > newDist) {
          nl.nodeB.distance = newDist;
        }        
      }
      
      this._commandQueue.add(new NodeVisitCommand(this, current));
    }
  }
  
  // @brief Walks the shortest path from the given node and sets the color of the nodes of that path. 
  public void tracePath(Node current){
    if(null != current){
      
      current.setVisited(true);
      if(current != this.getDestination() && current != this.getSource()){
        current.setColor(0.5, 0.5, 1.0, 1.0);
      }
      float minDist = current.distance;
      Node minNode = null;
      
      // Process neighbours.
      for (NodeLink nl : current.links) {
        if (nl.nodeB.distance < minDist && !nl.nodeB.getVisited()) {
          minDist = nl.nodeB.distance;
          minNode = nl.nodeB;
         
        }
      }
      
      if(null != minNode){
          this.tracePath(minNode);
      }
    }
  }
  
  // @brief Walks the shortest path from the source and sets the color of the nodes of that path. 
  //        Unvisits all nodes prior to traversing.
  public void tracePath(){
    this.unvisitAll();
    this.tracePath(this.getSource());
  }
  
  public boolean isDone(){
    return this._isDone;
  }

  public Node[] getNodes() {
    return this._nodes;
  }

  public Node getNode(float x, float y) {
    int ix = int(x);
    int iy = int(y);
    return this.getNode(ix, iy);
  }

  public Node getNode(int x, int y) {
    int pos = x + y * this._gridWt;
    if (pos < 0 || pos > this._gridWt * this._gridHt) {
      return null;
    } 
    return _nodes[pos];
  }

  public boolean setNode(float x, float y, Node n) {
    int ix = int(x);
    int iy = int(y);
    return this.setNode(ix, iy, n);
  }

  public boolean setNode(int x, int y, Node n) {
    int pos = x + y * this._gridWt;
    if (pos < 0 || pos > this._gridWt * this._gridHt) {
      return false;
    } 
    _nodes[pos] = n;
    return true;
  }

  public Node getDestination() {
    return this._destination;
  }

  // @brief sets the current destination.
  // @return the old destination.
  public Node setDestination(int x, int y) {
    Node old = this.getDestination();
    Node node = this.getNode(x, y);

    if (null != node) {
      this._destination = node;
    }

    return old;
  }

  // @brief sets the current destination.
  // @return the old destination.
  public Node setDestination(float x, float y) {
    int ix = int(x);
    int iy = int(y);
    return this.setDestination(ix, iy);
  }

  // @brief sets the current destination.
  // @return the old destination.
  public Node setDestination(Node n) {
    Node old = this.getDestination();
    this._destination = n;
    return old;
  }



  public Node getSource() {
    return this._source;
  }

  // @brief sets the current source.
  // @return the old source.
  public Node setSource(int x, int y) {
    Node old = this.getSource();
    Node node = this.getNode(x, y);

    if (null != node) {
      this._source = node;
    }

    return old;
  }

  // @brief sets the current source.
  // @return the old source.
  public Node setSource(float x, float y) {
    int ix = int(x);
    int iy = int(y);
    return this.setSource(ix, iy);
  }

  // @brief sets the current source.
  // @return the old source.
  public Node setSource(Node n) {
    Node old = this.getSource();
    this._source = n;
    return old;
  }
}

class NodeCommand{
  private Node target;
  private Dijkstra grid;
  
  public NodeCommand(Dijkstra g, Node t){
    this.grid = g;
    this.target = t;
  }
  
  public void run(){} 
}

class NodeVisitCommand extends NodeCommand{
  public NodeVisitCommand(Dijkstra g, Node t){
    super(g, t);
  }
  @Override
  public void run(){
    super.grid.visit(super.target);
  }
}

class NodeTentativeCommand extends NodeCommand{
  public NodeTentativeCommand(Dijkstra g, Node t){
    super(g, t);
  }
  @Override
  public void run(){
    super.grid.tentative(super.target);
  }
}

class NodeLink {
  public Node nodeB;
  public float weight;
  public NodeLink() {
  }
}

class NodeLinkManager {
  public void createForNodes(Node a, Node b, float weight) {
    NodeLink nl = new NodeLink();
    nl.nodeB = b;
    nl.weight = weight;
    a.links.add(nl);
  }
}

class Node {
  static final public float SIZE = 10;
  static final private float DEFAULT_COLOR_R = 0.5;
  static final private float DEFAULT_COLOR_G = 0.5;
  static final private float DEFAULT_COLOR_B = 0.5;
  static final private float DEFAULT_COLOR_A = 1.0;

  public float x;
  public float y;
  public float distance = 0.0;
  public ArrayList<NodeLink> links;
  private float[] _color;
  private boolean _visited = false;
  private boolean _shouldRedraw = false;

  public Node(float _x, float _y) {
    this._color = new float[]{DEFAULT_COLOR_R, DEFAULT_COLOR_G, DEFAULT_COLOR_B, DEFAULT_COLOR_A};
    this.x = _x;
    this.y = _y;
    this.links = new ArrayList();
    _shouldRedraw = true;
  }

  public void setColor(float r, float g, float b, float a) {
    this._color = new float[]{r, g, b, a};
    _shouldRedraw = true;
  }

  public boolean eq(Node n) {
    return null != n && this.x == n.x && this.y == n.y;
  }
  
  public boolean getVisited(){
    return this._visited;
  }
  
  public void setVisited(boolean v){
     this._visited = v; 
  }

  public void render() {
    if(this._shouldRedraw){
      fill(this._color[0], this._color[1], this._color[2], this._color[3]);
      rect(this.x * Node.SIZE + MASTER_OFFSET_X, this.y * Node.SIZE + MASTER_OFFSET_Y, Node.SIZE, Node.SIZE);
      _shouldRedraw = false;
    }
  }
}
