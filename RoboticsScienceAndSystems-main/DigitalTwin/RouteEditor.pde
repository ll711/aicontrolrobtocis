/** Mouse route editor and acknowledged command transport. World storage is mm;
 * only ROUTE_POINT values are converted to metres. No edit starts the robot. */
class RouteEditor_c {
  final int MAX_POINTS = 256;
  final float MAX_LENGTH_MM = 4000;
  final float MAX_COORDINATE_MM = 10000;
  final int COMMAND_TIMEOUT_MS = 2500;
  final int HEARTBEAT_MS = 500;
  final int IDLE = 0, UPLOAD_STOP = 1, UPLOAD_BEGIN = 2, UPLOAD_POINT = 3,
    UPLOAD_END = 4, STARTING = 5, STOPPING = 6, RESETTING = 7;
  Simulator sim;
  ArrayList<PVector> points = new ArrayList<PVector>();
  ArrayList<ArrayList<PVector>> undo = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> uploadPoints = new ArrayList<PVector>();
  ArrayList<PVector> trail = new ArrayList<PVector>();
  ArrayList<PVector> simulationTrail = new ArrayList<PVector>();
  int simLoadedRevision = -1;
  boolean freehand = false, drawing = false, movedPoint = false;
  boolean uploaded = false, wasConnected = false, robotRunning = false;
  boolean panning = false;
  Client sessionClient;
  int panMouseX, panMouseY;
  int dragged = -1, revision = 0, uploadedRevision = -1, phase = IDLE;
  int pointIndex = 0, sentAt = 0, lastPingAt = 0, connectedAt = 0, lastReplyAt = -1;
  String pending = "", message = "Draw a route. Editing never starts the robot.";
  String firmwareStatus = "Physical robot: offline", physicalState = "OFFLINE";
  String lastFailure = "";

  RouteEditor_c(Simulator owner) { sim = owner; }

  boolean connected() { return !sim.simulationMode && sim.telemetryClient != null && sim.telemetryClient.active(); }
  boolean editable() {
    if (sim.simulationMode) return !sim.controller.running;
    return !connected() || (phase == IDLE && !robotRunning);
  }
  boolean canStart() {
    if (sim.simulationMode) return !sim.controller.running && !drawing && dragged < 0
      && simLoadedRevision == revision && sim.controller.core.ready();
    return connected() && phase == IDLE && !robotRunning && !drawing && dragged < 0
      && physicalState.equals("WAITING") && uploaded
      && uploadedRevision == revision && lastReplyAt >= 0
      && millis() - lastReplyAt < COMMAND_TIMEOUT_MS;
  }

  void update() {
    if (sim.simulationMode) {
      RobotPose_c p = sim.robot.odometry.pose;
      if (simulationTrail.isEmpty() || dist(p.x,p.y,simulationTrail.get(simulationTrail.size()-1).x,
          simulationTrail.get(simulationTrail.size()-1).y) >= 1) {
        simulationTrail.add(new PVector(p.x,p.y));
        if(simulationTrail.size()>5000) simulationTrail.remove(0);
      }
      return;
    }
    boolean online = connected();
    int now = millis();
    if (online != wasConnected || (online && sessionClient != sim.telemetryClient)) {
      uploaded = false; uploadedRevision = -1; phase = IDLE; pending = "";
      robotRunning = false; drawing = false; dragged = -1;
      if (online) {
        connectedAt = now; lastReplyAt = -1; lastPingAt = now - HEARTBEAT_MS;
        physicalState = "CONNECTED";
        message = "Connected. Upload this route before Start is available.";
      } else {
        physicalState = "OFFLINE";
        message = "Disconnected: Start disabled. Reconnect and upload again.";
        firmwareStatus = "Offline; robot watchdog stops after 3 seconds without commands.";
      }
      wasConnected = online;
      sessionClient = sim.telemetryClient;
    }
    if (!online) return;
    if (now - lastPingAt >= HEARTBEAT_MS) {
      lastPingAt = now;
      sim.telemetryClient.write("PING\n");
    }
    if (phase != IDLE && now - sentAt > COMMAND_TIMEOUT_MS) {
      fail("No ACK for " + pending + "; STOP requested");
      return;
    }
    int replyReference = lastReplyAt >= 0 ? lastReplyAt : connectedAt;
    if (now - replyReference > COMMAND_TIMEOUT_MS) {
      uploaded = false;
      sim.telemetryClient.write("STOP\n");
      sim.telemetryClient.stop();
      message = "Robot replies timed out; disconnected and Start disabled.";
    }
  }

  void send(String command, String expected, int nextPhase) {
    if (!connected()) { message = "Offline: command not sent."; return; }
    pending = expected; phase = nextPhase; sentAt = millis();
    sim.telemetryClient.write(command + "\n");
  }

  void fail(String reason) {
    uploaded = false; uploadedRevision = -1; lastFailure = reason;
    message = reason;
    if (connected() && phase != STOPPING) send("STOP", "STOP", STOPPING);
    else { phase = IDLE; pending = ""; }
  }

  boolean handleLine(String line, int now) {
    if (!(line.startsWith("ACK ") || line.startsWith("ERR ") || line.startsWith("STATUS ")))
      return false;
    lastReplyAt = now;
    if (line.startsWith("STATUS ")) {
      firmwareStatus = line;
      if (line.indexOf("mode=RUNNING") >= 0) { robotRunning = true; physicalState = "RUNNING"; }
      if (line.indexOf("mode=WAITING") >= 0 || line.indexOf("mode=STOPPED") >= 0) {
        robotRunning = false;
        physicalState = line.indexOf("mode=WAITING") >= 0 ? "WAITING" : "STOPPED";
      }
      if (line.indexOf("route_ready=0") >= 0) uploaded = false;
      return true;
    }
    if (line.startsWith("ERR ")) { fail(line); return true; }
    String[] fields = splitTokens(line);
    if (fields.length < 2 || fields[1].equals("PING")) return true;
    if (phase == IDLE || !fields[1].equals(pending)) return true;
    if (phase == UPLOAD_BEGIN || phase == UPLOAD_POINT || phase == UPLOAD_END) {
      int expectedCount = phase == UPLOAD_POINT ? pointIndex + 1 : uploadPoints.size();
      if (fields.length != 3 || !fields[2].equals(str(expectedCount))) {
        fail("Unexpected ACK count: " + line); return true;
      }
    }
    int completed = phase;
    phase = IDLE; pending = "";
    if (completed == UPLOAD_STOP) {
      robotRunning = false; physicalState = "WAITING";
      send("ROUTE_BEGIN " + uploadPoints.size(), "ROUTE_BEGIN", UPLOAD_BEGIN);
    } else if (completed == UPLOAD_BEGIN) {
      pointIndex = 0; sendNextPoint();
    } else if (completed == UPLOAD_POINT) {
      pointIndex++;
      if (pointIndex < uploadPoints.size()) sendNextPoint();
      else send("ROUTE_END", "ROUTE_END", UPLOAD_END);
    } else if (completed == UPLOAD_END) {
      uploaded = true; uploadedRevision = revision;
      physicalState = "WAITING";
      message = "Route accepted (" + uploadPoints.size() + " points). Press Start explicitly.";
    } else if (completed == STARTING) {
      robotRunning = true; physicalState = "RUNNING";
      message = "Physical robot started. STOP remains available.";
    } else if (completed == STOPPING) {
      robotRunning = false; physicalState = "WAITING";
      message = lastFailure.length() > 0 ? lastFailure + " | STOP acknowledged." : "STOP acknowledged.";
      lastFailure = "";
    } else if (completed == RESETTING) {
      robotRunning = false; physicalState = "WAITING"; trail.clear();
      message = "Pose reset acknowledged: (0,0), heading +X. Place the robot to match.";
    }
    return true;
  }

  void sendNextPoint() {
    PVector point = uploadPoints.get(pointIndex);
    String x = String.format(java.util.Locale.US, "%.6f", point.x / 1000.0);
    String y = String.format(java.util.Locale.US, "%.6f", point.y / 1000.0);
    send("ROUTE_POINT " + x + " " + y, "ROUTE_POINT", UPLOAD_POINT);
    message = "Uploading point " + (pointIndex + 1) + " / " + uploadPoints.size() + " ...";
  }

  void upload() {
    String problem = validationError();
    if (sim.simulationMode) {
      if(problem.length()>0) { message=problem;return; }
      sim.robot.setMotorPWM(0,0);
      boolean loaded=sim.controller.loadRoute(points);
      simLoadedRevision=loaded?revision:-1;
      message=loaded?"Simulation route loaded. Press START.":sim.controller.status;
      return;
    }
    if (!connected()) { message = "Join the robot WiFi first; TCP is offline."; return; }
    if (phase != IDLE) { message = "Wait for the current command, or press STOP."; return; }
    if (problem.length() > 0) { message = problem; return; }
    uploaded = false; uploadedRevision = -1; lastFailure = "";
    uploadPoints = copyPoints(points);
    send("STOP", "STOP", UPLOAD_STOP);
    message = "Stopping the robot before uploading ...";
  }

  void stop() {
    if (sim.simulationMode) {
      sim.controller.reset();sim.robot.setMotorPWM(0,0);
      drawing=false;dragged=-1;message="Simulation stopped.";return;
    }
    if (!connected()) { message = "Offline: STOP cannot be acknowledged; robot uses its watchdog."; return; }
    if (phase >= UPLOAD_STOP && phase <= UPLOAD_END) uploaded = false;
    drawing = false; dragged = -1; lastFailure = "";
    send("STOP", "STOP", STOPPING);
    message = "STOP requested; waiting for the robot acknowledgement.";
  }

  void onTelemetry(TCPTelemetry_c sample) {
    if (trail.size() == 0 || dist(sample.x, sample.y, trail.get(trail.size()-1).x,
        trail.get(trail.size()-1).y) >= 1.0) {
      trail.add(new PVector(sample.x, sample.y));
      if (trail.size() > 3000) trail.remove(0);
    }
  }

  ArrayList<PVector> copyPoints(ArrayList<PVector> source) {
    ArrayList<PVector> result = new ArrayList<PVector>();
    for (PVector point : source) result.add(point.copy());
    return result;
  }
  void remember() {
    undo.add(copyPoints(points));
    if (undo.size() > 30) undo.remove(0);
  }
  void changed() {
    revision++; uploaded = false; uploadedRevision = -1; simLoadedRevision = -1;
    message = "Draft changed. Upload required before Start.";
  }
  float routeLength() {
    float result = 0;
    for (int i = 1; i < points.size(); i++) result += PVector.dist(points.get(i-1), points.get(i));
    return result;
  }
  String validationError() {
    if (points.size() < 2) return "Add at least two different route points.";
    if (points.size() > MAX_POINTS) return "Route exceeds 256 points; simplify or undo.";
    if (routeLength() < 1) return "Route needs at least 1 mm of length.";
    if (routeLength() > MAX_LENGTH_MM) return "Route exceeds 4.0 m; shorten it before uploading.";
    for (PVector p : points) {
      if (Float.isNaN(p.x) || Float.isNaN(p.y) || Float.isInfinite(p.x) || Float.isInfinite(p.y)
        || abs(p.x) > MAX_COORDINATE_MM || abs(p.y) > MAX_COORDINATE_MM)
        return "Each route coordinate must be within -10 m to +10 m.";
    }
    return "";
  }

  float panelX() { return sim.uiMargin(); }
  float panelWidth() { return sim.statusWidth(); }
  int columns() { return panelWidth() >= 600 ? 4 : 2; }
  float panelHeight() { return 146 + (12 / columns()) * 34; }
  float panelTop() { return height - panelHeight() - sim.uiMargin(); }
  boolean insideCanvas(float x, float y) {
    float top = sim.uiMargin() + max(18, 16 * sim.uiScale()) * 5 + 18;
    float right = sim.showTelemetry ? width - sim.telemetryPanelWidth() : width;
    return x >= 0 && x < right && y > top && y < panelTop() - 6;
  }
  PVector world(float x, float y) {
    return new PVector((x - width / 2.0) / sim.zoom + sim.viewCentreX,
      (height / 2.0 - y) / sim.zoom + sim.viewCentreY);
  }
  int hitPoint(PVector location) {
    for (int i = points.size()-1; i >= 0; i--)
      if (PVector.dist(points.get(i), location) <= 10 / sim.zoom) return i;
    return -1;
  }

  void mousePressed() {
    int button = buttonAt(mouseX, mouseY);
    if (button >= 0) { if (mouseButton == LEFT) activate(button); return; }
    if (!insideCanvas(mouseX, mouseY)) return;
    if (mouseButton == CENTER) {
      panning = true; panMouseX = mouseX; panMouseY = mouseY;
      sim.trackRobot = false; return;
    }
    if (!editable()) { message = "Stop the physical robot before editing."; return; }
    sim.trackRobot = false;
    PVector point = world(mouseX, mouseY);
    if (mouseButton == RIGHT) {
      int hit = hitPoint(point);
      if (hit >= 0) { remember(); points.remove(hit); changed(); }
      return;
    }
    if (mouseButton != LEFT) return;
    if (freehand) {
      remember(); drawing = true; changed(); addStrokePoint(point);
    } else {
      int hit = hitPoint(point);
      if (hit >= 0) { dragged = hit; movedPoint = false; return; }
      if (points.size() >= MAX_POINTS) { message = "256-point limit reached. Undo or clear."; return; }
      remember(); points.add(point); changed();
    }
  }

  void mouseDragged() {
    if (panning) {
      sim.panBy((panMouseX-mouseX)/sim.zoom, (mouseY-panMouseY)/sim.zoom);
      panMouseX = mouseX; panMouseY = mouseY;
      return;
    }
    if (!editable() || !insideCanvas(mouseX, mouseY)) return;
    PVector point = world(mouseX, mouseY);
    if (drawing) { addStrokePoint(point); return; }
    if (dragged >= 0) {
      if (!movedPoint) { remember(); changed(); movedPoint = true; }
      points.set(dragged, point);
    }
  }
  void addStrokePoint(PVector point) {
    if (points.size() >= 2048) { message = "Stroke buffer full; release to simplify."; return; }
    if (points.size() == 0 || PVector.dist(points.get(points.size()-1), point) >= 3 / sim.zoom)
      points.add(point);
  }
  void mouseReleased() {
    if (drawing) {
      float tolerance = 2.0;
      ArrayList<PVector> raw = points;
      points = simplify(raw, tolerance);
      while (points.size() > MAX_POINTS && tolerance < 512) {
        tolerance *= 1.4;
        points = simplify(raw, tolerance);
      }
      message = "Freehand stroke simplified (" + nf(tolerance, 0, 1)
        + " mm tolerance). Review the blue route before uploading.";
    }
    drawing = false; dragged = -1; movedPoint = false; panning = false;
  }

  // Iterative Ramer-Douglas-Peucker: retains endpoints without deep recursion.
  ArrayList<PVector> simplify(ArrayList<PVector> input, float tolerance) {
    if (input.size() < 3) return copyPoints(input);
    boolean[] keep = new boolean[input.size()];
    keep[0] = keep[input.size()-1] = true;
    ArrayList<int[]> stack = new ArrayList<int[]>();
    stack.add(new int[] {0, input.size()-1});
    while (stack.size() > 0) {
      int[] range = stack.remove(stack.size()-1);
      PVector a = input.get(range[0]), b = input.get(range[1]);
      float vx = b.x-a.x, vy = b.y-a.y, length2 = vx*vx + vy*vy;
      int furthest = -1; float best = tolerance*tolerance;
      for (int i = range[0]+1; i < range[1]; i++) {
        PVector p = input.get(i);
        float t = length2 == 0 ? 0 : constrain(((p.x-a.x)*vx + (p.y-a.y)*vy)/length2, 0, 1);
        float dx = p.x-a.x-t*vx, dy = p.y-a.y-t*vy, d2 = dx*dx+dy*dy;
        if (d2 > best) { best = d2; furthest = i; }
      }
      if (furthest >= 0) {
        keep[furthest] = true;
        stack.add(new int[] {range[0], furthest}); stack.add(new int[] {furthest, range[1]});
      }
    }
    ArrayList<PVector> result = new ArrayList<PVector>();
    for (int i = 0; i < input.size(); i++) if (keep[i]) result.add(input.get(i).copy());
    return result;
  }

  void mouseWheel(float amount) {
    if (!insideCanvas(mouseX, mouseY) || drawing || dragged >= 0) return;
    PVector before = world(mouseX, mouseY);
    sim.zoom = constrain(sim.zoom * pow(1.15, -amount), sim.minimumZoom, sim.maximumZoom);
    PVector after = world(mouseX, mouseY);
    sim.viewCentreX += before.x - after.x; sim.viewCentreY += before.y - after.y;
    sim.trackRobot = false;
  }

  boolean handleKey(char character, int keyCode) {
    if (character == ' ') { stop(); return true; }
    if (character == 'h' || character == 'H') {
      sim.viewCentreX = sim.viewCentreY = 0; sim.trackRobot = false; return true;
    }
    if (character == 'z' || character == 'Z') { activate(2); return true; }
    return false;
  }

  float buttonWidth() { return (panelWidth() - 20 - (columns()-1)*8) / columns(); }
  int buttonAt(float x, float y) {
    for (int i = 0; i < 12; i++) {
      float bx = panelX()+10+(i%columns())*(buttonWidth()+8);
      float by = panelTop()+52+(i/columns())*34;
      if (x >= bx && x <= bx+buttonWidth() && y >= by && y <= by+28) return i;
    }
    return -1;
  }
  boolean enabled(int index) {
    if(index>=8) return phase==IDLE && !robotRunning && !sim.controller.running && !drawing && dragged<0;
    if (index < 4) return editable() && !drawing && dragged < 0;
    if (sim.simulationMode) {
      if(index==4) return !drawing && dragged<0 && validationError().length()==0;
      if(index==5) return canStart();
      if(index==6) return true;
      return !sim.controller.running;
    }
    if (index == 4) return connected() && phase == IDLE && !drawing && dragged < 0
      && validationError().length() == 0;
    if (index == 5) return canStart();
    if (index == 6) return connected();
    return connected() && phase == IDLE && !robotRunning;
  }
  void activate(int index) {
    if (!enabled(index)) { message = "Action unavailable: check connection, route and robot state."; return; }
    if (index == 0) { freehand = false; message = "Points: left-click to add, drag to move, right-click a point to delete."; }
    else if (index == 1) { freehand = true; message = "Freehand mode: hold the left mouse button and draw."; }
    else if (index == 2) {
      if (undo.size() > 0) { points = undo.remove(undo.size()-1); changed(); }
    } else if (index == 3) { remember(); points.clear(); changed(); }
    else if (index == 4) upload();
    else if (index == 5) {
      if(sim.simulationMode) { sim.controller.setSignal(1);message="Simulation running. STOP or Space stops it."; }
      else { send("START", "START", STARTING); message = "START requested ..."; }
    }
    else if (index == 6) stop();
    else if (index == 7) {
      if(sim.simulationMode) {
        sim.controller.reset();sim.robot.resetForRun(0,0,0,0,0);simulationTrail.clear();
        message="Simulation pose reset to origin, heading +X.";
      } else {
        send("RESET_POSE", "RESET_POSE", RESETTING);
        message = "Resetting physical pose to (0,0), heading +X ...";
      }
    } else if(index==8 || index==9) {
      if(!sim.simulationMode && sim.telemetryClient!=null) {
        sim.telemetryClient.write("STOP\n");sim.telemetryClient.stop();sim.telemetryClient=null;
      }
      sim.controller.reset();sim.robot.setMotorPWM(0,0);
      sim.simulationMode=index==8;uploaded=false;uploadedRevision=-1;wasConnected=false;
      simLoadedRevision=-1;physicalState="OFFLINE";
      message=sim.simulationMode?"Simulation selected. Load route, then START.":"Physical robot selected. Connect WiFi, upload route, then START.";
    } else if(index==10) {
      remember();points.clear();
      points.add(new PVector(0,0));points.add(new PVector(200,0));
      points.add(new PVector(200,160));points.add(new PVector(0,160));points.add(new PVector(0,0));
      changed();message="Square demo ready. Reset pose, load route, then START.";
    } else if(index==11) {
      sim.viewCentreX=sim.viewCentreY=0;sim.zoom=1;sim.trackRobot=false;
    }
  }

  void drawWorld() {
    pushStyle();
    strokeWeight(1/sim.zoom); stroke(90, 115, 135, 100);
    line(-60, 0, 100, 0); line(0, -60, 0, 100);
    stroke(65, 105, 180); line(100, 0, 90, 5); line(100, 0, 90, -5);
    stroke(225, 125, 35, 170); strokeWeight(2/sim.zoom); noFill();
    beginShape(); for (PVector point : trail) vertex(point.x, point.y); endShape();
    stroke(125,65,190);strokeWeight(2/sim.zoom);
    beginShape(); for(PVector point:simulationTrail) vertex(point.x,point.y);endShape();
    boolean accepted=sim.simulationMode?simLoadedRevision==revision:uploaded && uploadedRevision==revision;
    stroke(accepted ? color(30, 145, 125) : color(20, 105, 230));
    strokeWeight(3/sim.zoom); noFill();
    beginShape(); for (PVector point : points) vertex(point.x, point.y); endShape();
    for (int i = 0; i < points.size(); i++) {
      PVector point = points.get(i);
      fill(i == 0 ? color(35, 175, 100) : (i == points.size()-1 ? color(225, 70, 85) : color(245)));
      ellipse(point.x, point.y, 4/sim.zoom, 4/sim.zoom);
    }
    popStyle();
  }

  String fitText(String source, float available) {
    if (textWidth(source) <= available) return source;
    while (source.length() > 0 && textWidth(source + "...") > available)
      source = source.substring(0, source.length()-1);
    return source + "...";
  }
  void drawPanel() {
    float x = panelX(), y = panelTop(), w = panelWidth();
    pushStyle(); rectMode(CORNER); noStroke(); fill(248, 251, 255, 245);
    rect(x, y, w, panelHeight(), 8);
    fill(25, 45, 70); textSize(15); textAlign(LEFT, BASELINE);
    text("ROUTE EDITOR  |  " + (sim.simulationMode?"SIMULATION - "+sim.controller.status:"PHYSICAL - "+physicalState), x+10, y+20);
    textSize(12);
    String problem = validationError();
    fill(problem.length() > 0 && points.size() >= 2 ? color(185, 65, 40) : color(65));
    text(fitText(points.size() + "/256 points   " + nf(routeLength()/1000.0, 0, 3)
      + "/4.000 m   coordinates -10..+10 m   "
      + (sim.simulationMode?(simLoadedRevision==revision?"LOADED":"DRAFT"):(uploaded?"UPLOADED":"DRAFT")), w-20), x+10, y+39);
    String[] labels = {"Points", "Freehand", "Undo (Z)", "Clear", sim.simulationMode?"Load into sim":"Upload route",
      "START", "STOP (Space)", "Reset pose", "Simulation", "Physical TCP", "Square demo", "Reset view"};
    for (int i = 0; i < labels.length; i++) {
      float bx = x+10+(i%columns())*(buttonWidth()+8), by = y+52+(i/columns())*34;
      boolean active = enabled(i), selected = (i == 0 && !freehand) || (i == 1 && freehand)
        || (i==8 && sim.simulationMode) || (i==9 && !sim.simulationMode);
      fill(!active ? color(222, 227, 234) : (i == 6 ? color(190, 55, 55) :
        (selected ? color(35, 105, 190) : color(220, 235, 250))));
      rect(bx, by, buttonWidth(), 28, 5);
      fill(active && (i == 6 || selected) ? color(255) : (active ? color(30, 55, 85) : color(130)));
      textAlign(CENTER, CENTER); text(labels[i], bx+buttonWidth()/2, by+13);
    }
    textAlign(LEFT, BASELINE);
    float textY = y+52+(12/columns())*34+14;
    fill(35, 55, 75); text(fitText(message, w-20), x+10, textY);
    String details=sim.simulationMode?"Tree1: "+sim.controller.decision.name()+" | PWM L/R: "
      +(int)sim.robot.motors.left.pwm+" / "+(int)sim.robot.motors.right.pwm+" | progress "
      +sim.controller.decision.progress+" / "+sim.controller.core.sampleCount():firmwareStatus;
    fill(90); text(fitText(details, w-20), x+10, textY+17);
    PVector cursor = world(mouseX, mouseY);
    // Keep command/fault messages visible above, and explain why an otherwise
    // drawn route cannot be uploaded instead of only colouring its length red.
    String help = problem.length() > 0 && points.size() >= 2 ? problem :
      "Left: add/move | Right: delete | Middle drag: pan | Wheel: zoom | H: origin";
    if (problem.length() > 0 && points.size() >= 2) fill(185, 65, 40);
    text(fitText(help, w-20), x+10, textY+34);
    fill(90);
    text(fitText("Mouse (m): " + nf(cursor.x/1000.0, 0, 3) + ", " + nf(cursor.y/1000.0, 0, 3)
      + " | Origin (0,0), heading +X; reset pose only when physically aligned.", w-20), x+10, textY+51);
    popStyle();
  }
}
