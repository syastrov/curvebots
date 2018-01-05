import * as Paper from 'paper';
import { Bot } from './bot';
import { curveCommand, Curve, Direction } from '../messages';
import * as d3Voronoi from 'd3-voronoi';

class VoronoiBot extends Bot {
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


    let v = d3Voronoi.voronoi();
    let points:[number, number][] = [];
    
    let l = new data.paper.Layer();
    let curvesGroup = new data.paper.Group();
    for (const curve of data.curves) {
      let path = <Paper.CompoundPath>curve.path;
      curvesGroup.addChild(path);
    }

    for (const curve of data.curves) {
      let path = <Paper.CompoundPath>curve.path;
      for (const child of path.children) {
        const path = <Paper.Path>child;
        const pathLength = path.length;
        const interval = 25;
        if (pathLength === 0) {
          continue;
        } 
        for (let i = 0; i += interval; i <= pathLength) {
          const point = path.getPointAt(i);
          if (point === null) {
            break;
          }
          points.push([point.x, point.y]);
        }
      }
    }

    const bounds = data.paper.view.bounds;
    v.extent([[bounds.left, bounds.top], [bounds.right, bounds.bottom]]);
    let diagram = v(points);
    let polys = diagram.polygons();

    // let g = new data.paper.Group();
    for (const poly of polys) {
      let p = new data.paper.Path(poly);
      // g.addChild(p);
    }
    // let p = new data.paper.Path(diagram.edges);
    
    let goodEdges: [[number, number], [number, number]][] = [];
    for (const edge of diagram.edges) {
      if (!edge) continue;
      const start = edge[0];
      const end = edge[1];
      const a = new data.paper.Path.Line({from: start, to: end});
      let intersects = false;
      for (const curve of data.curves) {
        let path = <Paper.CompoundPath>curve.path;
        if (a.intersects(path)) {
          intersects = true;
          break;
        }
      }
      if (intersects) {
        continue;
      }
      // let activeLayer = data.paper.project.activeLayer;
      // this.debugLayer.addChild(a);
      // activeLayer.activate();
      goodEdges.push(edge);
    }

    // Try to find the nearest line to us
    let c = new data.paper.CompoundPath({});
    let shortestDistance = Infinity;
    let nearestPoint:Paper.Point|null = null;
    for (const edge of goodEdges) {
      let p = new data.paper.Path({
        segments: [edge[0], edge[1]]
      });
      c.addChild(p);
      const pathNearest = p.getNearestPoint(data.pos);
      const dist = pathNearest.getDistance(data.pos);
      if (dist < shortestDistance) {
        nearestPoint = pathNearest;
        shortestDistance = dist;
      }
    }

    if (nearestPoint) {
      let circle = new data.paper.Path.Circle(nearestPoint, 10);
      this.debugLayer.addChild(circle);
      // Try to turn towards the nearest line
      const angle = data.pos.getDirectedAngle(nearestPoint);
      if (angle < 0) {
        command = -1;
      } else {
        command = 1;
      }
      console.log("Got nearestPoint", shortestDistance, angle, data.direction.deg);
    } else {
      console.log("No nearest");
    }
    this.debugLayer.addChild(c);

    

    // ([[1, 1], [2, 2]]);
    // debugger;
    // let curve = this.getMyCurve(data.curves);
    // let pos = new data.paper.Point(curve.pos);
    // let direction = new data.paper.Point(curve.direction.x, curve.direction.y);
    // const turningRadius = 29.443664472000638; // derived empirically


    this.sendPaintMessage(data.paper.project);
    this.sendCommand(id, command);
  }
}

new VoronoiBot();
