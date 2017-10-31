package map;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

import javax.swing.JComponent;
import javax.swing.JFrame;

import map.BotClientMap.Wall.WallType;

public class BotClientMap {
  public int gridSize;
  public Pose startPose;
  public ArrayList<Wall> walls;
  public ArrayList<Stack> stacks;
  public HomeBase base;

  public BotClientMap() {
    walls = new ArrayList<>();
    stacks = new ArrayList<>();
    base = new HomeBase();
  }

  public void load(String s) {
    String[] parts = s.split("\n");
    int i = 0;
    // this.gridSize = parseGridSize(parts[i++]);
    this.startPose = parsePose(parts[i++]);

    for (; i < parts.length; i++) {
      if (parts[i].charAt(0) == 'W' ||
          parts[i].charAt(0) == 'D') {
        walls.add(parseWall(parts[i]));
      }
      else if (parts[i].charAt(0) == 'S') {
        stacks.add(parseStack(parts[i]));
      }
      else if (parts[i].charAt(0) == 'H') {
        base = parseHomeBase(parts[i]);
      }
      else {
        // not handled yet
      }
    }
  }

  // private int parseGridSize(String s) {
  //   return Integer.valueOf(s);
  // }

  private Pose parsePose(String s) {
    String[] parts = s.split(",");
    assert(parts[0] == "L");
    return new Pose(Integer.valueOf(parts[1].trim()), Integer.valueOf(parts[2].trim()), Pose.Heading.valueOf(parts[3].trim()));
  }

  private Wall parseWall(String s) {
    String[] parts = s.split(",");

    Point start = new Point(Integer.valueOf(parts[1].trim()),
                            Integer.valueOf(parts[2].trim()));
    Point end = new Point(Integer.valueOf(parts[3].trim()),
                          Integer.valueOf(parts[4].trim()));
    Wall.WallType type = Wall.WallType.values()[Wall.WallTypeShort.valueOf(
        parts[0]).ordinal()];

    return new Wall(start, end, type);
  }

  private Stack parseStack(String s) {
    String[] parts = s.split(",");

    Point loc = new Point(Integer.valueOf(parts[1].trim()),
                          Integer.valueOf(parts[2].trim()));
    Stack.BlockType c1 = Stack.BlockType.values()[Stack.BlockTypeShort.valueOf(
        parts[3]).ordinal()];
    Stack.BlockType c2 = Stack.BlockType.values()[Stack.BlockTypeShort.valueOf(
        parts[4]).ordinal()];
    Stack.BlockType c3 = Stack.BlockType.values()[Stack.BlockTypeShort.valueOf(
        parts[5]).ordinal()];

    return new Stack(loc, c1, c2, c3);
  }

  private HomeBase parseHomeBase(String s) {
    String[] parts = s.split(",");

    int n = Integer.valueOf(parts[1].trim());
    Point last = new Point(Integer.valueOf(parts[2*n].trim()),
                           Integer.valueOf(parts[2*n+1].trim()));
    HomeBase hb = new HomeBase();
    for (int i = 1; i <= n; ++i) {
      Point next = new Point(Integer.valueOf(parts[2*i].trim()),
                             Integer.valueOf(parts[2*i+1].trim()));
      hb.bounds.add(new Wall(last, next, WallType.WALL));
      last = next;
    }
    assert(hb.bounds.size() == n);
    return hb;
  }
  
  public static class Point {
    public final int x;
    public final int y;

    public Point(int x, int y) {
      this.x = x;
      this.y = y;
    }

    @Override
    public String toString() {
      return String.format("(%d, %d)", x, y);
    }

    public String toBotClientString() {
      return String.format("%d,%d", x, y);
    }
  }

  public static class Pose extends Point {
    public enum Heading {
      N,E,S,W;
      private static Heading[] vals = values();
      public Heading next()
      {
        return vals[(this.ordinal()+1) % vals.length];
      }
    }

    public final Heading h;

    public Pose(int x, int y, Heading h) {
      super(x, y);
      this.h = h;
    }

    @Override
    public String toString() {
      return String.format("(%d,%d,%s)", x, y, h);
    }

    public String toBotClientString() {
      return String.format("L,%d,%d,%s", x, y, h);
    }
  }

  public static class Wall {
    enum WallTypeShort {
      W, D
    };

    enum WallType {
      WALL, DISPENSER
    };

    public final WallType type;
    public final Point start;
    public final Point end;

    public Wall(Point start, Point end, WallType type) {
      this.start = start;
      this.end = end;
      this.type = type;
    }

    @Override
    public String toString() {
      return String.format("Wall: %s\t[%s - %s]", type, start, end);
    }

    public String toBotClientString() {
      return String.format("%s,%s,%s",
                           WallTypeShort.values()[type.ordinal()],
                           start.toBotClientString(),
                           end.toBotClientString());
    }
  }

  public static class HomeBase {
    public static ArrayList<Wall> bounds;

    public HomeBase() {
      bounds = new ArrayList<Wall>();
    }

    @Override
    public String toString() {
      String out = String.format("Home base: (%d) ", bounds.size());
      Point last = null;
      for (Wall w : bounds) {
        if (last != null) {
          assert(w.start.x == last.x);
          assert(w.start.y == last.y);
        }
        out += w.start;
        out += ",";
        last = w.end;
      }
      assert(last.x == bounds.get(0).start.x);
      assert(last.y == bounds.get(0).start.y);
      return out;
    }

    public String toBotClientString() {
      String out = String.format("H,%d", bounds.size());
      Point last = null;
      for (Wall w : bounds) {
        if (last != null) {
          assert(w.start.x == last.x);
          assert(w.start.y == last.y);
        }
        out += "," + w.start.toBotClientString();
        last = w.end;
      }
      assert(last.x == bounds.get(0).start.x);
      assert(last.y == bounds.get(0).start.y);
      return out;
    }
  }

  public static class Stack {
    enum BlockTypeShort {
      R, G
    };

    enum BlockType {
      RED, GREEN
    };

    public final Point loc;
    public BlockType c1;
    public BlockType c2;
    public BlockType c3;

    public Stack(Point loc, BlockType c1, BlockType c2, BlockType c3) {
      this.loc = loc;
      this.c1 = c1;
      this.c2 = c2;
      this.c3 = c3;
    }

    @Override
    public String toString() {
      return String.format("Wall: [%s]\t%s,%s,%s", loc, c1, c2, c3);
    }

    public String toBotClientString() {
      return String.format("S,%s,%s,%s,%s",
                           loc.toBotClientString(),
                           BlockTypeShort.values()[c1.ordinal()],
                           BlockTypeShort.values()[c2.ordinal()],
                           BlockTypeShort.values()[c3.ordinal()]);
    }
  }

  @Override
  public String toString() {
    // String mapString = String.format("Grid Size: %.2f\n", gridSize);
    String mapString = "";
    mapString += "Pose: " + this.startPose.toString();
    for (Wall w : walls)
      mapString += "\n" + w.toString();
    for (Stack s : stacks)
      mapString += "\n" + s.toString();
    return mapString;
  }

  public String toBotClientString() {
    // String mapString = String.format("%.2f:", gridSize);
    String mapString = "";
    mapString += startPose.toBotClientString() + "\n";
    mapString += base.toBotClientString() + "\n";
    for (Wall w : walls)
      mapString += w.toBotClientString() + "\n";
    for (Stack s : stacks) 
      mapString += s.toBotClientString() + "\n";
    return mapString;
  }

  public static BotClientMap getDefaultMap() {
    // String mapString = "22.00:4.00,6.00,2.36:1.00,3.00,1.00,4.00,N:1.00,4.00,0.00,5.00,N:0.00,5.00,0.00,6.00,N:0.00,6.00,1.00,6.00,N:1.00,6.00,1.00,7.00,N:1.00,7.00,1.00,8.00,N:1.00,8.00,2.00,8.00,R:2.00,8.00,4.00,8.00,S:4.00,8.00,5.00,7.00,N:5.00,7.00,6.00,6.00,N:6.00,6.00,5.00,5.00,N:5.00,5.00,6.00,4.00,N:6.00,4.00,5.00,3.00,R:5.00,3.00,4.00,3.00,N:4.00,3.00,4.00,4.00,N:4.00,4.00,4.00,5.00,N:4.00,5.00,3.00,4.00,N:3.00,4.00,3.00,3.00,N:3.00,3.00,2.00,3.00,N:2.00,3.00,1.00,3.00,R:";
    String mapString = "L,5,5,N\nH,7,7,5,7,4,7,3,6,3,5,3,5,4,6,5\nW,5,3,4,4\nW,4,4,3,5\nW,3,5,3,6\nW,3,6,3,7\nW,3,7,3,8\nW,3,8,4,9\nW,4,9,5,9\nW,5,9,6,9\nD,6,9,6,8\nW,6,8,6,7\nD,6,7,6,6\nW,6,6,5,6\nW,5,6,6,5\nW,6,5,7,6\nW,7,6,8,6\nW,8,6,8,5\nW,8,5,8,4\nW,8,4,7,3\nW,7,3,6,3\nW,6,3,5,3\nS,7,5,G,G,R\nS,6,4,R,R,R\nS,4,6,G,R,G\nS,4,8,G,G,G\nS,5,7,G,R,R";

    BotClientMap m = new BotClientMap();
    m.load(mapString);
    return m;
  }

  public void drawMap() {
    JFrame jf = new JFrame();
    jf.setContentPane(new MapPainter());
    jf.setSize(800, 800);
    jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    jf.setBackground(Color.white);
    jf.setVisible(true);
  }

  private class MapPainter extends JComponent {
    private Point start = null;
    private int size = 50;
    private int xOff = 1 * size;
    private int yOff = 15 * size;

    public MapPainter() {
      this.setFocusable(true);
      this.addMouseListener(new MouseAdapter() {
          @Override
          public void mouseClicked(MouseEvent e) {
            super.mouseClicked(e);
            Point p = toPoint(e.getPoint());
            if (start == null) {
              start = p;
            } else {
              int dx = start.x - p.x;
              int dy = start.y - p.y;
              if (dx * dx + dy * dy <= 2) {
                Wall w = new Wall(start, p, WallType.WALL);
                walls.add(w);
                start = null;
              } else {
                System.err.println("Error: Invalid wall length!");
              }
            }

            repaint();
          }
        });

      this.addKeyListener(new KeyAdapter() {
          @Override
          public void keyPressed(KeyEvent e) {
            super.keyPressed(e);
            
            try {
  
              WallType t = null;
              Stack s = null;
              switch (e.getKeyChar()) {
                case 'w':
                  t = WallType.WALL;
                  break;
                case 'l':
                  t = WallType.DISPENSER;
                  break;
                case 'h':
                  if (walls.size() > 0) {
                    Wall b = walls.remove(walls.size() - 1);
                    base.bounds.add(b);
                  } else {
                    throw new Exception("Need an existing wall before converting it to a homebase line.");
                  }
                  break;
                case 'd':
                  if (walls.size() > 0)
                    walls.remove(walls.size() - 1);
                  else {
                    throw new Exception("No more walls to undo.");
                  }
                  break;
                case 'x':
                  if (stacks.size() > 0)
                    stacks.remove(stacks.size() - 1);
                  else {
                    throw new Exception("No more stacks to undo.");
                  }
                  break;
                case 'c':
                  if (base.bounds.size() > 0) {
                    base.bounds.clear();
                  } else {
                    throw new Exception("Homebase polygon already empty.");
                  }
                  break;
                case 'p':
                  System.out.println("--------------- THE MAP ------------------");
                  System.out.println("-------- DO NOT USE THESE DASHES ---------\n");
                  System.out.println(toBotClientString());
                  System.out.println("-------- DO NOT USE THESE DASHES ---------");
                  break;
                case 's':
                  if (start != null) {
                    s = new Stack(start, Stack.BlockType.GREEN,
                                  Stack.BlockType.GREEN, Stack.BlockType.GREEN);
                    stacks.add(s);
                    start = null;
                  } else {
                    throw new Exception("Can't create a stack without a point.");
                  }
                  break;
                case '1':
                  if (stacks.size() > 0) {
                    Stack last = stacks.get(stacks.size() - 1);
                    if (last.c1 == Stack.BlockType.RED)
                      last.c1 = Stack.BlockType.GREEN;
                    else
                      last.c1 = Stack.BlockType.RED;
                  }
                  break;
                case '2':
                  if (stacks.size() > 0) {
                    Stack last = stacks.get(stacks.size() - 1);
                    if (last.c2 == Stack.BlockType.RED)
                      last.c2 = Stack.BlockType.GREEN;
                    else
                      last.c2 = Stack.BlockType.RED;
                  }
                  break;
                case '3':
                  if (stacks.size() > 0) {
                    Stack last = stacks.get(stacks.size() - 1);
                    if (last.c3 == Stack.BlockType.RED)
                      last.c3 = Stack.BlockType.GREEN;
                    else
                      last.c3 = Stack.BlockType.RED;
                  }
                  break;
                case 'i':
                  if (start != null) {
                    startPose = new Pose(start.x, start.y, Pose.Heading.N);
                    start = null;
                  } else {
                    throw new Exception("Can't set initial position without a point.");
                  }
                  break;
                case 'r':
                  startPose = new Pose(startPose.x, startPose.y, startPose.h.next());
                  break;
              }
  
              if (t != null) {
                if (walls.size() > 0) {
                  Wall w = walls.remove(walls.size() - 1);
                  walls.add(new Wall(w.start, w.end, t));
                } else {
                  throw new Exception("Can't set wall type [" + t.name() + "] without an existing wall!");
                }
              }
            } catch (Exception exc) {
              System.err.println("Error: " + exc.getMessage());
            }

            repaint();
          }
        });
    }

    @Override
    public void paint(Graphics g) {
      super.paint(g);
      ((Graphics2D) g).setStroke(new BasicStroke(3));

      g.setColor(Color.gray);
      for (int x = 0; x < 20; x++) {
        for (int y = 0; y < 20; y++) {
          g.fillOval(toPixelX(x) - 1, toPixelY(y) - 1, 3, 3);
        }
      }

      Color[] wallColors = new Color[] { Color.blue, new Color(240,230,0) };
      for (Wall w : walls) {
        g.setColor(wallColors[w.type.ordinal()]);
        g.drawLine(toPixelX(w.start.x), toPixelY(w.start.y),
                   toPixelX(w.end.x), toPixelY(w.end.y));
      }
      
      if (start != null) {
        g.setColor(Color.magenta);
        g.fillOval(toPixelX(start.x) - 3, toPixelY(start.y) - 3, 7, 7);
      }

      g.setColor(Color.magenta);
      for (Wall b : base.bounds) {
        g.drawLine(toPixelX(b.start.x), toPixelY(b.start.y),
                   toPixelX(b.end.x), toPixelY(b.end.y));
      }

      Color[] stackColors = new Color[] { Color.red, new Color(0,180,0) };
      for (Stack s : stacks) {
        g.setColor(stackColors[s.c1.ordinal()]);
        g.fillRect(toPixelX(s.loc.x)-3, toPixelY(s.loc.y)-3, 7, 7);
        g.setColor(stackColors[s.c2.ordinal()]);
        g.fillRect(toPixelX(s.loc.x)-3, toPixelY(s.loc.y)-11, 7, 7);
        g.setColor(stackColors[s.c3.ordinal()]);
        g.fillRect(toPixelX(s.loc.x)-3, toPixelY(s.loc.y)-19, 7, 7);
      }

      g.setColor(Color.black);
      Polygon robot = getRobotPolygon(startPose);
      g.fillPolygon(robot);

      g.setColor(Color.black);
      g.drawString("W - wall", 20, 20);
      g.drawString("L - dispenser", 20, 35);
      g.drawString("H - homebase", 20, 50);
      g.drawString("P - print", 20, 65);
      
      g.drawString("S - stack", 120, 20);
      g.drawString("1 - toggle bot", 120, 35);
      g.drawString("2 - toggle mid", 120, 50);
      g.drawString("3 - toggle top", 120, 65);
      
      g.drawString("I - initial position", 220, 20);
      g.drawString("R - rotate initial position", 220, 35);
      g.drawString("D - remove last wall or dispenser", 220, 50);
      g.drawString("X - remove last stack", 220, 65);
      g.drawString("C - remove all homebases", 220, 80);
    }

    public int toPixelX(double x) {
      return (int) (x * size) + xOff;
    }

    public int toPixelY(double y) {
      return (int) (-y * size) + yOff;
    }

    public Polygon getRobotPolygon(Pose pose) {
      int[] xCoords = new int[3];
      int[] yCoords = new int[3];

      switch (pose.h) {
        // I really didn't want to figure out rotating polygons...
        case N:
          xCoords[0] = toPixelX((double)pose.x - 0.2);
          xCoords[1] = toPixelX((double)pose.x);
          xCoords[2] = toPixelX((double)pose.x + 0.2);
          yCoords[0] = toPixelY((double)pose.y - 0.3);
          yCoords[1] = toPixelY((double)pose.y + 0.3);
          yCoords[2] = toPixelY((double)pose.y - 0.3);
          break;
        case E:
          xCoords[0] = toPixelX((double)pose.x - 0.3);
          xCoords[1] = toPixelX((double)pose.x + 0.3);
          xCoords[2] = toPixelX((double)pose.x - 0.3);
          yCoords[0] = toPixelY((double)pose.y - 0.2);
          yCoords[1] = toPixelY((double)pose.y);
          yCoords[2] = toPixelY((double)pose.y + 0.2);
          break;
        case S:
          xCoords[0] = toPixelX((double)pose.x - 0.2);
          xCoords[1] = toPixelX((double)pose.x);
          xCoords[2] = toPixelX((double)pose.x + 0.2);
          yCoords[0] = toPixelY((double)pose.y + 0.3);
          yCoords[1] = toPixelY((double)pose.y - 0.3);
          yCoords[2] = toPixelY((double)pose.y + 0.3);
          break;
        case W:
          xCoords[0] = toPixelX((double)pose.x + 0.3);
          xCoords[1] = toPixelX((double)pose.x - 0.3);
          xCoords[2] = toPixelX((double)pose.x + 0.3);
          yCoords[0] = toPixelY((double)pose.y - 0.2);
          yCoords[1] = toPixelY((double)pose.y);
          yCoords[2] = toPixelY((double)pose.y + 0.2);
          break;
      }
      return new Polygon(xCoords, yCoords, 3);
    }

    public Point toPoint(java.awt.Point mousePoint) {
      double x = (mousePoint.x - xOff) / (double) size;
      double y = -(mousePoint.y - yOff) / (double) size;
      
      return new Point((int)Math.round(x), (int)Math.round(y));
    }
  }

  public static void main(String[] args) {
    BotClientMap map = getDefaultMap();
    map.drawMap();
  }
}
