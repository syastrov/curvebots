import * as Paper from 'paper';
import { Bot } from './bot';
import { curveCommand, Curve, Direction, Point, TYPE } from '../messages';
import * as d3Voronoi from 'd3-voronoi';
import { CurvePainter } from '../shared';
// import { Set } from 'es6-shim';

// interface Set<T> {
//   add(value: T): Set<T>;
//   clear(): void;
//   delete(value: T): boolean;
//   entries(): Array<[T, T]>;
//   forEach(callbackfn: (value: T, index: T, set: Set<T>) => void, thisArg?: any): void;
//   has(value: T): boolean;
//   keys(): Array<T>;
//   size: number;
// }



// interface SetConstructor {
//   new <T>(): Set<T>;
//   new <T>(iterable: Array<T>): Set<T>;
//   prototype: Set<any>;
// }
// declare var Set: SetConstructor;

interface DijkstraResult<T> {
  dist: Map<T, number>;
  prev: Map<T, T | null>;
}

class Graph<T> {
  getEdgesFrom(node: T): T[] {
    return this._edges.has(node) ? <T[]>(this._edges).get(node) : [];
  }

  _edges: Map<T, T[]>;
  _nodes: T[];
  constructor() {
    this._nodes = [];
    this._edges = new Map();
  }

  addNode(n: T) {
    this._nodes.push(n);
    this._edges.set(n, []);
  }

  addEdge(n1: T, n2: T) {
    let edge = this._edges.get(n1);
    if (!edge) {
      return false;
    }
    edge.push(n2);
    return true;
  }

  getNodes(): T[] {
    return this._nodes;
  }

  dijkstra(initialNode: T): DijkstraResult<T> {
    const nodes = this.getNodes();

    // Set distance to 0 for initial, and infinity for all other nodes
    let dist = new Map<T, number>();
    let prev = new Map<T, T | null>();
    for (const node of nodes) {
      dist.set(node, Infinity);
      prev.set(node, null);
    }
    // debugger;
    let currentNode: T = initialNode;
    dist.set(currentNode, 0);

    const unvisited = new Set(nodes);

    const findNodeWithMinDistance = (unvisited: Set<T>) => {
      // Inefficient
      let minDist = Infinity;
      let minNode: T | null = null;

      const unvisitedIterable = unvisited.keys();
      let iter;
      while (iter = unvisitedIterable.next(), !iter.done) {
        const n = <T>iter.value;
        const distOfN = dist.get(n);
        if (distOfN === undefined) {
          continue;
        }
        if (distOfN <= minDist) {
          minDist = distOfN;
          minNode = n;
        }
      }
      return minNode;
    };

    while (unvisited.size > 0) {
      unvisited.delete(currentNode);
      // debugger;
      // debugger;
      for (const neighbor of this.getEdgesFrom(currentNode)) {
        if (!unvisited.has(neighbor)) {
          // Skip neighbors that have been visited.
          continue;
        }
        let alt = <number>dist.get(currentNode) + 1; // TODO: Weights
        if (alt < <number>dist.get(neighbor)) {
          // shorter path to neighbor
          dist.set(neighbor, alt);
          prev.set(neighbor, currentNode);
        }
      }
      currentNode = <T>findNodeWithMinDistance(unvisited);// node in unvisited with min dist[currentNode]; 
    }

    return { dist: dist, prev: prev };
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

  add(q: Conf) {
    return new Conf(this.x + q.x, this.y + q.y, this.angle + q.angle);
  }

  multiply(f: number) {
    return new Conf(this.x * f, this.y * f, this.angle * f);
  }
}

const enum ExtendResult {
  REACHED = 'reached',
  ADVANCED = 'advanced',
  TRAPPED = 'trapped'
}

interface RRTParams {
  reachedGoalDistanceThreshold: number;
  delta_t: number;
  speed: number
  turningRadius: number,
  useRungeKutta: boolean
}

class RRT {
  params: RRTParams;
  curvesGroup: Paper.Group;
  paper: typeof Paper;

  constructor(params: RRTParams, paper, curvesGroup) {
    this.params = params;
    this.paper = paper;
    this.curvesGroup = curvesGroup;
  }

  /**
   * 
   * @param q_init Initial configuration
   * @param K number of vertices 
   * @param delta_t incremental distance
   */
  generate_rrt(q_init: Conf, K: number): Graph<Conf> {
    // T.init(q_init)
    let G = new Graph<Conf>();
    G.addNode(q_init);
    for (let k = 1; k <= K; k++) {
      const q_rand = this.random_conf();
      const result = this.extend(G, q_rand);
    }
    return G;
  }
  extend(G: Graph<Conf>, q: Conf): ExtendResult {
    const q_near = this.nearest_neighbor(q, G);
    const result = this.new_conf(q_near, q);
    if (this.checkCollision(q_near, result[0], result[1])) {
      return ExtendResult.TRAPPED;
      // continue;
    }
    if (!result) {
      return ExtendResult.TRAPPED;
    }
    const [q_new, u_new] = result;

    G.addNode(q_new);
    // TODO: Add u_new to edge.
    G.addEdge(q_near, q_new);
    if (this.distance(q_new, q) <= this.params.reachedGoalDistanceThreshold) {
      return ExtendResult.REACHED;
    } else {
      return ExtendResult.ADVANCED;
    }
  }

  new_conf(q: Conf, u: Conf): [Conf, curveCommand] {
    // http://msl.cs.uiuc.edu/~lavalle/cs576_1999/projects/junqu/
    // Nonholonomic

    // Calculate new position after some delta time
    // moving from configuration q to configuration u.
    // There must be no collisions on the path between the two configurations

    // Try each possible input, and pick the one that takes us closest to destination.



    // If we can't determine what to do return null
    let bestResult: any = null;
    let shortestDistance = Infinity;
    for (let command: curveCommand = -1; command <= 1; command++) {
      const curveCommand = <curveCommand>command;
      const newConf = this.next_state(q, curveCommand);

      // TODO: Check for collisions and continue
      const distanceToNewConf = this.distance(newConf, u);
      if (distanceToNewConf < shortestDistance) {
        shortestDistance = distanceToNewConf;
        bestResult = <[Conf, curveCommand]>[newConf, command];
      }
    }

    return bestResult;
  }

  next_state(q: Conf, command: curveCommand): any {
    if (this.params.useRungeKutta) {
      const derivatives = (q: Conf, command: number) => {
        const speed = this.params.speed;
        const xDot = speed * Math.cos(q.angle);
        const yDot = speed * Math.sin(q.angle);
        const r = this.params.turningRadius;
        const thetaDot = speed * Math.tan((1 / r) * command);
        // const thetaDot = Math.acos(1 - (speed * speed) / (2 * r * r)) * command;
        return new Conf(xDot, yDot, thetaDot);
      }

      // Fourth-order Runge-Kutta
      let k1 = derivatives(q, command);
      let k2 = derivatives(q.add(k1.multiply(.5)), command);
      let k3 = derivatives(q.add(k2.multiply(.5)), command);
      let k4 = derivatives(q.add(k3), command);

      const sumK = k1.add(k2.multiply(2)).add(k3.multiply(2)).add(k4);
      return q.add(sumK.multiply(this.params.delta_t / 6));
    } else {
      const r = 30;
      const s = this.params.delta_t;
      const deltaAngle = Math.acos(1 - (s * s) / (2 * r * r)) * command;
      const newAngle = q.angle + deltaAngle;

      const newX = q.x + (Math.cos(newAngle) * s);
      const newY = q.y + (Math.sin(newAngle) * s);

      return new Conf(newX, newY, newAngle);
    }
  }

  checkCollision(q1: Conf, q2: Conf, command: curveCommand): boolean {
    let painter = new CurvePainter(this.paper, -1, new this.paper.Color('#0f0'));
    // TODO: Detect collisions with previous parts of own path
    painter.startSegment({ x: q1.x, y: q1.y });
    painter.addToSegment({ x: q2.x, y: q2.y }, command);

    const point = new this.paper.Point([q2.x, q2.y]);
    const strokeWidth = painter.path.strokeWidth;
    // Bounds
    if (!point.isInside(this.paper.view.bounds.expand(-strokeWidth))) {
      painter.path.remove();
      return true;
    }

    // Grab our last path
    const lastPath = <Paper.Path>painter.path.lastChild;
    // lastPath.strokeWidth = 1;
    // if (lastPath.intersects(this.curvesGroup.children)) {
    // painter.path.remove();
    // debugger;
    // return true;
    // }
    // if (!lastPath) 
    // const line = new this.paper.Path.Line({ from: [q1.x, q1.y], to: [q2.x, q2.y] });
    for (const curve of this.curvesGroup.children) {
      const path = <Paper.CompoundPath>curve;
      if (lastPath.getIntersections(path,
        (a: Paper.CurveLocation) => {
          // Ignore hitting ourself
          return a.curveOffset > 0;
        }
      ).length) {
        // line.remove();
        painter.path.remove();
        return true;
      }
    }
    // line.remove();
    // painter.path.remove();
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
      (q1.x - q2.x) ** 2 + (q1.y - q2.y) ** 2 +
      Math.min((q1.angle - q2.angle) ** 2 +
        (q1.angle - q2.angle + 2 * Math.PI) ** 2 +
        (q1.angle - q2.angle - 2 * Math.PI) ** 2)
    )
  }

  nearest_neighbor(q: Conf, G: Graph<Conf>): Conf {
    let min = Infinity;
    const vertices = G.getNodes();
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
  counter: number;
  path: Array<Conf>;
  paperPath: Paper.Path
  constructor() {
    super();
    this.counter = 0;
    this.path = [];
  }
  update(id: number, data: {
    paper: typeof Paper,
    curves: Curve[],
    bounds: Paper.Path,
    pos: Paper.Point,
    direction: Direction
  }) {
    let command: curveCommand = 0;
    let params: RRTParams = {
      reachedGoalDistanceThreshold: 50,
      delta_t: 9,
      speed: 1,
      turningRadius: 30,
      useRungeKutta: false
    };

    this.counter++;

    if (this.counter % 1 === 0) {
      this.counter = 0;
      this.path = [];
    }

    if (!this.path.length) {
      let rrt = new RRT(params, data.paper, this.curvesGroup);
      const G = rrt.generate_rrt(
        new Conf(data.pos.x, data.pos.y, data.direction.rad),
        200
      );
      const initialNode = G.getNodes()[0];
      let result = G.dijkstra(initialNode);
      let iterable = result.dist.entries();
      let iter = iterable.next();
      let maxDist = 0;
      let farthestNode: Conf | null = null;
      while (!iter.done) {
        let [conf, dist] = iter.value;
        if (dist >= maxDist) {
          maxDist = dist;
          farthestNode = conf;
        }
        iter = iterable.next();
      }


      if (farthestNode) {
        // Walk back to build a path
        let curNode: Conf | null | undefined = farthestNode;
        do {
          this.path.push(curNode);
          curNode = result.prev.get(curNode);
        } while (curNode);
        this.path.reverse();
        console.log(this.path);
        this.paperPath = new data.paper.Path(this.path.map(p => [p.x, p.y]));
      }
    }

    this.paperPath.strokeColor = '#fff';
    this.paperPath.parent = this.debugLayer;



    // Try to turn towards the next configuration in the path.
    if (this.path.length >= 2) {
      const nextConf = this.path[1];
      const nextPos = new data.paper.Point(nextConf.x, nextConf.y);
      const circle = new data.paper.Path.Circle({ center: nextPos, radius: 2, parent: this.debugLayer });
      // TODO: Somehow, determine whether the angle changes too much, and ignore it.
      const angleToNext = 180 * nextConf.angle / Math.PI - data.direction.deg;
      console.log(angleToNext);
      if (Math.abs(angleToNext) < 0.05) {
        command = 0;
      } else
        if (angleToNext > 0) {
          command = 1;
        } else {
          command = -1;
        }
    }

    this.sendPaintMessage(data.paper.project);
    this.sendCommand(id, command);
  }
}

new RRTBot();
