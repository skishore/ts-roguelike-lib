import {assert, int, only} from './lib';

//////////////////////////////////////////////////////////////////////////////
// Simple 2D geometry helpers.

class Point {
  readonly x: int;
  readonly y: int;

  static origin = new Point(0, 0);

  constructor(x: int, y: int) {
    this.x = x | 0;
    this.y = y | 0;
  }

  add(o: Point): Point { return new Point(this.x + o.x, this.y + o.y); }
  sub(o: Point): Point { return new Point(this.x - o.x, this.y - o.y); }

  equal(o: Point): boolean { return this.x === o.x && this.y === o.y; }

  // An injection from Z x Z -> Z suitable for use as a Map key.
  key(): int {
    const {x, y} = this;
    const ax = x < 0 ? -2 * x + 1 : 2 * x;
    const ay = y < 0 ? -2 * y + 1 : 2 * y;
    const n = ax + ay;
    return n * (n + 1) / 2 + ax;
  }

  toString(): string { return `Point(${this.x}, ${this.y})`; }
};

class Direction extends Point {
  static none = new Direction(0, 0);
  static n    = new Direction(0, -1);
  static ne   = new Direction(1, -1);
  static e    = new Direction(1, 0);
  static se   = new Direction(1, 1);
  static s    = new Direction(0, 1);
  static sw   = new Direction(-1, 1);
  static w    = new Direction(-1, 0);
  static nw   = new Direction(-1, -1);

  static all = [Direction.n, Direction.ne, Direction.e, Direction.se,
                Direction.s, Direction.sw, Direction.w, Direction.nw];

  static cardinal = [Direction.n, Direction.e, Direction.s, Direction.w];

  static diagonal = [Direction.ne, Direction.se, Direction.sw, Direction.nw];

  private constructor(x: int, y: int) { super(x, y); }

  static assert(point: Point): Direction {
    if (point.equal(Direction.none)) return Direction.none;
    return only(Direction.all.filter(x => x.equal(point)));
  }
};

class Matrix<T> {
  readonly size: Point;
  #data: T[];

  constructor(size: Point, value: T) {
    this.size = size;
    this.#data = Array(size.x * size.y).fill(value);
  }

  get(point: Point): T {
    if (!this.contains(point)) throw new Error(`${point} not in ${this.size}`);
    return this.#data[point.x + this.size.x * point.y]!;
  }

  set(point: Point, value: T): void {
    if (!this.contains(point)) throw new Error(`${point} not in ${this.size}`);
    this.#data[point.x + this.size.x * point.y] = value;
  }

  getOrNull(point: Point): T | null {
    if (!this.contains(point)) return null;
    return this.#data[point.x + this.size.x * point.y]!;
  }

  contains(point: Point): boolean {
    const {x: px, y: py} = point;
    const {x: sx, y: sy} = this.size;
    return 0 <= px && px < sx && 0 <= py && py < sy;
  }

  fill(value: T): void {
    this.#data.fill(value);
  }
};

//////////////////////////////////////////////////////////////////////////////
// Tran-Thong symmetric line-of-sight calculation.

const LOS = (a: Point, b: Point): Point[] => {
  const {x: xa, y: ya} = a;
  const {x: xb, y: yb} = b;
  const result: Point[] = [a];

  const x_diff = Math.abs(xa - xb);
  const y_diff = Math.abs(ya - yb);
  const x_sign = xb < xa ? -1 : 1;
  const y_sign = yb < ya ? -1 : 1;

  let test = 0;
  let [x, y] = [xa, ya];

  if (x_diff >= y_diff) {
    test = Math.floor((x_diff + test) / 2);
    for (let i = 0; i < x_diff; i++) {
      x += x_sign;
      test -= y_diff;
      if (test < 0) {
        y += y_sign;
        test += x_diff;
      }
      result.push(new Point(x, y));
    }
  } else {
    test = Math.floor((y_diff + test) / 2);
    for (let i = 0; i < y_diff; i++) {
      y += y_sign;
      test -= x_diff;
      if (test < 0) {
        x += x_sign;
        test += y_diff;
      }
      result.push(new Point(x, y));
    }
  }

  return result;
};

//////////////////////////////////////////////////////////////////////////////
// A-star, for finding a path from a source to a known target.

const AStarUnitCost = 16;
const AStarDiagonalPenalty = 2;
const AStarLOSDeltaPenalty = 1;
const AStarOccupiedPenalty = 64;

// "delta" penalizes paths that travel far from the direct line-of-sight
// from the source to the target. In order to compute it, we figure out if
// this line is "more horizontal" or "more vertical", then compute the the
// distance from the point to this line orthogonal to this main direction.
//
// Adding this term to our heuristic means that it's no longer admissible,
// but it provides two benefits that are enough for us to use it anyway:
//
//   1. By breaking score ties, we expand the fronter towards T faster than
//      we would with a consistent heuristic. We complete the search sooner
//      at the cost of not always finding an optimal path.
//
//   2. By biasing towards line-of-sight, we select paths that are visually
//      more appealing than alternatives (e.g. that interleave cardinal and
//      diagonal steps, rather than doing all the diagonal steps first).
//
const AStarHeuristic = (p: Point, los: Point[]): int => {
  const {x: px, y: py} = p;
  const {x: sx, y: sy} = los[0]!;
  const {x: tx, y: ty} = los[los.length - 1]!;

  const delta = (() => {
    const dx = tx - sx;
    const dy = ty - sy;
    const l = los.length - 1;
    if (Math.abs(dx) > Math.abs(dy)) {
      const index = dx > 0 ? px - sx : sx - px;
      if (index < 0) return Math.abs(px - sx) + Math.abs(py - sy);
      if (index > l) return Math.abs(px - tx) + Math.abs(py - ty);
      return Math.abs(py - los[index]!.y);
    } else {
      const index = dy > 0 ? py - sy : sy - py;
      if (index < 0) return Math.abs(px - sx) + Math.abs(py - sy);
      if (index > l) return Math.abs(px - tx) + Math.abs(py - ty);
      return Math.abs(px - los[index]!.x);
    }
  })();

  const x = Math.abs(tx - px);
  const y = Math.abs(ty - py);
  const min = Math.min(x, y);
  const max = Math.max(x, y);
  return AStarUnitCost * max +
         AStarDiagonalPenalty * min +
         AStarLOSDeltaPenalty * delta;
};

class AStarNode {
  public popped: boolean = false;
  constructor(public point: Point, public parent: AStarNode | null,
              public distance: int, public score: int) {}
};

// Min-heap implementation on lists of A* nodes. Nodes track indices as well.
type AStarHeap = AStarNode[];

const AStarHeapExtractMin = (heap: AStarHeap): AStarNode => {
  assert(heap.length > 0);
  let best_index: int | null = null;
  let best_score = Infinity;
  for (let i = 0; i < heap.length; i++) {
    const score = heap[i]!.score;
    if (score > best_score) continue;
    best_score = score;
    best_index = i;
  }

  assert(best_index !== null);
  const result = heap[best_index!]!;
  const popped = heap.pop()!;
  if (best_index! < heap.length) {
    heap[best_index!] = popped;
  }
  result.popped = true;
  return result;
};

enum Status { FREE, BLOCKED, OCCUPIED };

const AStar = (source: Point, target: Point, check: (p: Point) => Status,
               record?: Point[]): Point[] | null => {
  // Try line-of-sight - if that path is clear, then we don't need to search.
  // As with the full search below, we don't check if source is blocked here.
  const los = LOS(source, target);
  const free = (() => {
    for (let i = 1; i < los.length - 1; i++) {
      if (check(los[i]!) !== Status.FREE) return false;
    }
    return true;
  })();
  if (free) return los.slice(1);

  const map: Map<int, AStarNode> = new Map();
  const heap: AStarHeap = [];

  const score = AStarHeuristic(source, los);
  const node = new AStarNode(source, null, 0, score);
  map.set(source.key(), node);
  heap.push(node);

  while (heap.length > 0) {
    const cur_node = AStarHeapExtractMin(heap);
    const cur = cur_node.point;
    if (record) record.push(cur);

    if (cur.equal(target)) {
      let current = cur_node;
      const result: Point[] = [];
      while (current.parent) {
        result.push(current.point);
        current = current.parent;
      }
      return result.reverse();
    }

    for (const direction of Direction.all) {
      const next = cur.add(direction);
      const test = next.equal(target) ? Status.FREE : check(next);
      if (test === Status.BLOCKED) continue;

      const diagonal = direction.x !== 0 && direction.y !== 0;
      const occupied = test === Status.OCCUPIED;
      const distance = cur_node.distance + AStarUnitCost +
                       (diagonal ? AStarDiagonalPenalty : 0) +
                       (occupied ? AStarOccupiedPenalty : 0);

      const key = next.key();
      const existing = map.get(key);

      // index !== null is a check to see if we've already popped this node
      // from the heap. We need it because our heuristic is not admissible.
      //
      // Using such a heuristic substantially speeds up search in easy cases,
      // with the downside that we don't always find an optimal path.
      if (existing && !existing.popped && existing.distance > distance) {
        existing.score += distance - existing.distance;
        existing.distance = distance;
        existing.parent = cur_node;
      } else if (!existing) {
        const score = distance + AStarHeuristic(next, los);
        const created = new AStarNode(next, cur_node, distance, score);
        map.set(key, created);
        heap.push(created);
      }
    }
  }

  return null;
};

//////////////////////////////////////////////////////////////////////////////

export {Point, Direction, Matrix, LOS, AStar, Status};
