import * as Paper from 'paper';
import { Bot } from './bot';
import { curveCommand, Curve, Direction, Point, TYPE } from '../messages';
import * as d3Voronoi from 'd3-voronoi';
import { CurvePainter } from '../shared';

class Graph<T> {
  _edges: any;
  _vertices: T[];
  constructor() {
    this._vertices = [];
    this._edges = [];
  }

  addVertex(v: T) {
    this._vertices.push(v);
    this._edges[v] = [];
  }

  addEdge(v: T, u: T) {
    this._edges[v].push(u);
  }

  getVertices(): T[] {
    return this._vertices;
  }
}

class Conf {
  x: number;
  y: number;
  angle: number;
  constructor(x: number, y: number, angle: number) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }
}

const enum ExtendResult {
  REACHED = 'reached',
  ADVANCED = 'advanced',
  TRAPPED = 'trapped'
}

class RRT {
  curvesGroup: Paper.Group;
  paper: typeof Paper;

  constructor(paper, curvesGroup) {
    this.paper = paper;
    this.curvesGroup = curvesGroup;
  }

  /**
   * 
   * @param q_init Initial configuration
   * @param K number of vertices 
   * @param delta_q incremental distance
   */
  generate_rrt(q_init: Conf, K: number, delta_t: number): Graph<Conf> {
    // T.init(q_init)
    let G = new Graph<Conf>();
    G.addVertex(q_init);
    for (let k = 1; k <= K; k++) {
      const q_rand = this.random_conf();
      this.extend(G, q_rand, delta_t);
    }
    return G;
  }
  extend(G: Graph<Conf>, q: Conf, delta_t: number): ExtendResult {
    const q_near = this.nearest_neighbor(q, G);
    const result = this.new_conf(q_near, q, delta_t);
    if (!result) {
      return ExtendResult.TRAPPED;
    }
    const [q_new, u_new] = result;

    G.addVertex(q_new);
    // TODO: Add u_new to edge.
    G.addEdge(q_near, q_new);
    if (q_new === q) {
      return ExtendResult.REACHED;
    } else {
      return ExtendResult.ADVANCED;
    }
  }

  new_conf(q: Conf, u: Conf, delta_t: number): [Conf, curveCommand] | null {
    // http://msl.cs.uiuc.edu/~lavalle/cs576_1999/projects/junqu/
    // Nonholonomic

    // Calculate new position after some delta time
    // moving from configuration q to configuration u.
    // There must be no collisions on the path between the two configurations

    // Try each possible input, and pick the one that takes us closest to destination.

    const r = 30;
    const s = delta_t;

    // If we can't determine what to do, just go straight
    let bestResult: [Conf, curveCommand] | null = null;
    let shortestDistance = Infinity;
    for (let command: curveCommand = -1; command <= 1; command++) {
      const deltaAngle = Math.acos(1 - (s * s) / (2 * r * r)) * command;
      const newAngle = q.angle + deltaAngle;

      const newX = q.x + (Math.cos(newAngle) * s);
      const newY = q.y + (Math.sin(newAngle) * s);

      const newConf = new Conf(newX, newY, newAngle);
      // TODO: Check for collisions and continue
      if (this.checkCollision(q, newConf, <curveCommand>command)) {
        continue;
      }
      const distanceToNewConf = this.distance(newConf, u);
      if (distanceToNewConf < shortestDistance) {
        shortestDistance = distanceToNewConf;
        bestResult = <[Conf, curveCommand]>[newConf, command];
      }
    }

    return bestResult;
  }



  checkCollision(q1: Conf, q2: Conf, command: curveCommand): boolean {
    let painter = new CurvePainter(this.paper, 1, new this.paper.Color('#0f0'));
    // TODO: Detect collisions with previous parts of own path
    painter.startSegment({ x: q1.x, y: q1.y });
    painter.addToSegment({ x: q2.x, y: q2.y }, command);

    const point = new this.paper.Point([q2.x, q2.y]);
    const strokeWidth = painter.path.strokeWidth;
    // Bounds
    if (!point.isInside(this.paper.view.bounds.expand(-strokeWidth))) {
      return true;
    }

    // Grab our last path
    const lastPath = <Paper.Path>painter.path.lastChild;

    let hitTestGroup = new this.paper.Group();
    hitTestGroup.addChild(this.curvesGroup);
    // hitTestGroup.addChild(painter.path);
    // Check collision with all curves
    const hitResult = this.curvesGroup.hitTest(point, {
      stroke: true,
      tolerance: strokeWidth / 2,
      match: (hit) => {
        return true;
        if (hit.item.parent.data.type === TYPE.curve) {
          if (hit.location.path === lastPath) {
            // If it's our own curve, we ignore collisions with the tip.
            const offset = hit.location.offset;
            const curOffset = lastPath.length;
            const diff = curOffset - offset;
            if (diff <= 0.001) {
              // Ignore last part of own curve
              return false;
            }
          }
          return true;
        } else {
          return false;
        }
      }
    });

    if (hitResult) {
    painter.path.remove();
      return true;
    }

    return false;
  }


  random_conf(): Conf {
    const x = Math.random() * this.paper.view.bounds.width;
    const y = Math.random() * this.paper.view.bounds.height;
    const angle = Math.random() * Math.PI * 2;
    return new Conf(x, y, angle);
  }

  distance(q1: Conf, q2: Conf): number {
    // Factor in the direction, because turning is required
    // return Math.sqrt((q2.x - q1.x) ** 2 + (q2.y - q1.y) ** 2);
    // TODO: is this correct?
    return Math.sqrt(
      (q1.x - q2.x) ^ 2 + (q1.y - q2.y) ^ 2 +
      Math.min((q1.angle - q2.angle) ** 2 +
        (q1.angle - q2.angle + 2 * Math.PI) ** 2 +
        (q1.angle - q2.angle - 2 * Math.PI) ** 2)
    )
  }

  nearest_neighbor(q: Conf, G: Graph<Conf>): Conf {
    let min = Infinity;
    const vertices = G.getVertices();
    let nearestVertex = vertices[0]; // TODO: Acceptable default?
    for (const v of vertices) {
      // Calculate distance between q and v using measurement function
      const dist = this.distance(q, v);
      if (dist < min) {
        min = dist;
        nearestVertex = v;
      }
    }
    // Return nearest vertex.
    return nearestVertex;
  }
}

class RRTBot extends Bot {
  constructor() {
    super();
  }
  update(id: number, data: {
    paper: typeof Paper,
    curves: Curve[],
    bounds: Paper.Path,
    pos: Paper.Point,
    direction: Direction
  }) {
    let command: curveCommand = 0;

    let rrt = new RRT(data.paper, this.curvesGroup);
    const G = rrt.generate_rrt(
      new Conf(data.pos.x, data.pos.y, data.direction.rad),
      10,
      10
    );

    console.log(G);

    /// state transition equation: xdot = f(x, u)
    // vector u comes from a set U of inputs
    // vector xdot is the derivative of state with respect to time
    // This is a control-theoretic representation

    // By integrating function f over a fixed time interval delta_t:
    // The next state, x_new can be determined for initial state, x and input u (one of valid inputs)

    // By Euler integration, x_new ~= x + f(x, u)*delta_t
    // Preferable to use a higher-order integration technique,
    // e.g. Runge-Kutta

    // NEW_STATE(x, u, delta_t) returns x_new

    // command = -1;

    this.sendPaintMessage(data.paper.project);
    this.sendCommand(id, command);
  }
}

new RRTBot();
