import {assert, int} from './lib';

//////////////////////////////////////////////////////////////////////////////
// Simple 2D geometry helpers.

// Model a point a single, 30-bit unsigned integer. The lower 15 bits are the
// x value (plus kOffset, so that it's non-negative); the upper 15 bits are y.
const kOffset = 1 << 14;
const kOrigin = (kOffset << 15) + kOffset;

interface Point {__type__: 'Point'};

const Point = {
  origin: kOrigin as any as Point,
  make: (x: int, y: int) => kOrigin + x + (y << 15) as any as Point,

  x: (a: Point) => ((a as any as int) & 0x7fff) - kOffset,
  y: (a: Point) => (a as any as int >> 15) - kOffset,

  add(a: Point, b: Point): Point {
    return ((a as any as int) + ((b as any as int) - kOrigin)) as any as Point;
  },
  sub(a: Point, b: Point): Point {
    return ((a as any as int) - ((b as any as int) - kOrigin)) as any as Point;
  },
  equal(a: Point, b: Point): boolean {
    return (a as any as int) === (b as any as int);
  },
  key(point: Point): int {
    return point as any as int;
  },
  show(point: Point): string {
    const x = Point.x(point);
    const y = Point.y(point);
    return `Point(${x}, ${y})`;
  },
};

interface Direction extends Point {__subtype__: 'Direction'};

const Direction = {
  none: Point.make(0, 0)   as Direction,
  n:    Point.make(0, -1)  as Direction,
  ne:   Point.make(1, -1)  as Direction,
  e:    Point.make(1, 0)   as Direction,
  se:   Point.make(1, 1)   as Direction,
  s:    Point.make(0, 1)   as Direction,
  sw:   Point.make(-1, 1)  as Direction,
  w:    Point.make(-1, 0)  as Direction,
  nw:   Point.make(-1, -1) as Direction,

  all:      [] as Direction[],
  cardinal: [] as Direction[],
  diagonal: [] as Direction[],
};

Direction.all = [Direction.n, Direction.ne, Direction.e, Direction.se,
                 Direction.s, Direction.sw, Direction.w, Direction.nw];

Direction.cardinal = [Direction.n, Direction.e, Direction.s, Direction.w];

Direction.diagonal = [Direction.ne, Direction.se, Direction.sw, Direction.nw];

class Matrix<T> {
  readonly size: Point;
  private readonly data: T[];

  constructor(size: Point, value: T) {
    this.size = size;
    this.data = Array(Point.x(size) * Point.y(size)).fill(value);
  }

  get(point: Point): T {
    if (!this.contains(point)) throw new Error(`${point} not in ${this.size}`);
    return this.data[Point.x(point) + Point.x(this.size) * Point.y(point)]!;
  }

  set(point: Point, value: T): void {
    if (!this.contains(point)) throw new Error(`${point} not in ${this.size}`);
    this.data[Point.x(point) + Point.x(this.size) * Point.y(point)] = value;
  }

  getOrNull(point: Point): T | null {
    if (!this.contains(point)) return null;
    return this.data[Point.x(point) + Point.x(this.size) * Point.y(point)]!;
  }

  contains(point: Point): boolean {
    const px = Point.x(point);
    if (!(0 <= px && px < Point.x(this.size))) return false;
    const py = Point.y(point);
    if (!(0 <= py && py < Point.y(this.size))) return false;
    return true;
  }

  fill(value: T): void {
    this.data.fill(value);
  }
};

//////////////////////////////////////////////////////////////////////////////
// Tran-Thong symmetric line-of-sight calculation.

const LOS = (a: Point, b: Point): Point[] => {
  const xa = Point.x(a);
  const ya = Point.y(a);
  const xb = Point.x(b);
  const yb = Point.y(b);
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
      result.push(Point.make(x, y));
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
      result.push(Point.make(x, y));
    }
  }

  return result;
};

//////////////////////////////////////////////////////////////////////////////
// Quick and dirty hash table mapping u32 -> u32.

const kMissing = -1;
const kTombstone = -2;
const kMinCapacity  = 4;
const kMaxLoadFactor = 0.75;
const kUnusedBuffer = new Int32Array();

class HashTable {
  private size: int = -1;
  private capacity: int = -1;
  private max_size: int = -1;
  private keys: Int32Array = kUnusedBuffer;
  private vals: Int32Array = kUnusedBuffer;

  constructor(target: int = 0) {
    let capacity = kMinCapacity;
    while (capacity < target) capacity *= 2;
    this.init(capacity);
  }

  get(key: int): int | null {
    assert(key >= 0);
    const {keys, vals, capacity} = this;
    let current = this.hash(key) & (capacity - 1);
    for (let delta = 1; ; delta += 1) {
      const probe = keys[current]!;
      if (probe === key) return vals[current]!;
      if (probe === kMissing) return null;
      current = (current + delta) & (capacity - 1);
    }
  }

  set(key: int, val: int): void {
    assert(key >= 0);
    if (this.size === this.max_size) {
      this.rebuild(2 * this.capacity);
    }
    const {keys, vals, capacity} = this;
    let current = this.hash(key) & (capacity - 1);
    let tomb = -1;
    for (let delta = 1; ; delta += 1) {
      const probe = keys[current]!;
      if (probe === key) {
        vals[current] = val;
        return;
      }
      if (probe === kMissing) break;
      if (probe === kTombstone && tomb === -1) tomb = current;
      current = (current + delta) & (capacity - 1);
    }
    const target = tomb === -1 ? current : tomb;
    keys[target] = key;
    vals[target] = val;
    this.size++;
  }

  remove(key: int): void {
    assert(key >= 0);
    if (4 * this.size < this.max_size && this.capacity > kMinCapacity) {
      this.rebuild(this.capacity / 2);
    }
    const {keys, capacity} = this;
    let current = this.hash(key) & (capacity - 1);
    for (let delta = 1; ; delta += 1) {
      const probe = keys[current]!;
      if (probe === key) {
        keys[current] = kTombstone;
        this.size--;
        return;
      }
      if (probe === kMissing) return;
      current = (current + delta) & (capacity - 1);
    }
  }

  show(): string {
    const result: [int, int][] = [];
    const {keys, vals, capacity} = this;
    for (let i = 0; i < capacity; i++) {
      const key = keys[i]!;
      if (key < 0) continue;
      const val = vals[i]!;
      result.push([key, val]);
    }
    assert(result.length === this.size);
    result.sort((a, b) => a[0] - b[0]);
    const terms = result.map(([key, val]) => `  ${key}: ${val}`).join('\n');
    return `HashTable(size = ${this.size}):\n${terms}`;
  }

  private hash(key: int): int {
    let t0 = 0, v0 = 0x9dc5;
    let t1 = 0, v1 = 0x811c;

    v0 ^= (key & 0xff);
    t0 = v0 * 403; t1 = v1 * 403; t1 += v0<<8;
    v1 = (t1 + (t0 >>> 16)) & 0xffff; v0 = t0 & 0xffff;

    v0 ^= ((key >> 8) & 0xff);
    t0 = v0 * 403; t1 = v1 * 403; t1 += v0<<8;
    v1 = (t1 + (t0 >>> 16)) & 0xffff; v0 = t0 & 0xffff;

    v0 ^= ((key >> 16) & 0xff);
    t0 = v0 * 403; t1 = v1 * 403; t1 += v0<<8;
    v1 = (t1 + (t0 >>> 16)) & 0xffff; v0 = t0 & 0xffff;

    v0 ^= ((key >> 24) & 0xff);
    t0 = v0 * 403; t1 = v1 * 403; t1 += v0<<8;
    v1 = (t1 + (t0 >>> 16)) & 0xffff; v0 = t0 & 0xffff;

    return ((v1 & 0x3fff << 16) >>> 0) + v0;
  }

  private init(capacity: int) {
    this.size = 0;
    this.capacity = capacity;
    this.max_size = capacity * kMaxLoadFactor;
    this.keys = new Int32Array(capacity);
    this.vals = new Int32Array(capacity);
    this.keys.fill(kMissing);
  }

  private rebuild(target: int) {
    const {keys, vals, capacity} = this;
    this.init(target);
    for (let i = 0; i < capacity; i++) {
      const key = keys[i]!;
      if (key < 0) continue;
      const val = vals[i]!;
      this.set(key, val);
    }
  }
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
  const px = Point.x(p);
  const py = Point.y(p);
  const sx = Point.x(los[0]!);
  const sy = Point.y(los[0]!);
  const tx = Point.x(los[los.length - 1]!);
  const ty = Point.y(los[los.length - 1]!);

  const delta = (() => {
    const dx = tx - sx;
    const dy = ty - sy;
    const l = los.length - 1;
    if (Math.abs(dx) > Math.abs(dy)) {
      const index = dx > 0 ? px - sx : sx - px;
      if (index < 0) return Math.abs(px - sx) + Math.abs(py - sy);
      if (index > l) return Math.abs(px - tx) + Math.abs(py - ty);
      return Math.abs(py - Point.y(los[index]!));
    } else {
      const index = dy > 0 ? py - sy : sy - py;
      if (index < 0) return Math.abs(px - sx) + Math.abs(py - sy);
      if (index > l) return Math.abs(px - tx) + Math.abs(py - ty);
      return Math.abs(px - Point.x(los[index]!));
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
  constructor(public point: Point, public parent: AStarNode | null,
              public bits: int, public distance: int, public score: int) {}
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

  const kNumDirections = 8;
  assert(kNumDirections === Direction.all.length);
  const map: Map<int, AStarNode> = new Map();
  const heap: AStarHeap = [];

  const score = AStarHeuristic(source, los);
  const source_node = new AStarNode(source, null, 0, 0, score);
  map.set(Point.key(source), source_node);
  heap.push(source_node);

  while (heap.length > 0) {
    const prev_node = AStarHeapExtractMin(heap);
    const prev = prev_node.point;
    if (record) record.push(prev);

    if (Point.equal(prev, target)) {
      let current = prev_node;
      const result: Point[] = [];
      while (current.parent) {
        result.push(current.point);
        current = current.parent;
      }
      return result.reverse();
    }

    for (let i = 0; i < kNumDirections; i++) {
      const reverse_bit = 1 << ((i + kNumDirections / 2) % kNumDirections);
      if (prev_node.bits & reverse_bit) continue;
      const forward_bit = 1 << i;

      const direction = Direction.all[i]!;
      const next = Point.add(prev, direction);
      const test = Point.equal(next, target) ? Status.FREE : check(next);
      if (test === Status.BLOCKED) continue;

      const diagonal = i % 2 !== 0;
      const occupied = test === Status.OCCUPIED;
      const distance = prev_node.distance + AStarUnitCost +
                       (diagonal ? AStarDiagonalPenalty : 0) +
                       (occupied ? AStarOccupiedPenalty : 0);

      const key = Point.key(next);
      const existing = map.get(key);

      // The "reverse_bit unset" check excludes nodes that have been popped
      // from the heap. We need it because our heuristic is not admissible.
      //
      // Using such a heuristic substantially speeds up search in easy cases,
      // with the downside that we don't always find an optimal path.
      if (existing) {
        existing.bits |= forward_bit;
        if (existing.distance <= distance) continue;
        existing.score += distance - existing.distance;
        existing.distance = distance;
        existing.parent = prev_node;
      } else {
        const bits = forward_bit;
        const score = distance + AStarHeuristic(next, los);
        const next_node = new AStarNode(next, prev_node, bits, distance, score);
        map.set(key, next_node);
        heap.push(next_node);
      }
    }
  }

  return null;
};

//////////////////////////////////////////////////////////////////////////////

export {Point, Direction, Matrix, LOS, HashTable, AStar, Status};
