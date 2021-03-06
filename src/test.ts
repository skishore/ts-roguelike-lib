import {assert, bench, int, nonnull, only, range} from './lib';
import {Point, Matrix, HashTable, AStar, Status} from './geo';

//////////////////////////////////////////////////////////////////////////////

const testBasicPointOperations = () => {
  const o = Point.origin;
  const a = Point.make(3, 4);
  const b = Point.make(-4, 5);
  const c = Point.make(4, -7);

  assert(Point.x(o) === 0);
  assert(Point.y(o) === 0);
  assert(Point.x(a) === 3);
  assert(Point.y(a) === 4);
  assert(Point.x(b) === -4);
  assert(Point.y(b) === 5);
  assert(Point.x(c) === 4);
  assert(Point.y(c) === -7);

  assert(Point.x(Point.add(o, a)) === 3);
  assert(Point.y(Point.add(o, a)) === 4);
  assert(Point.x(Point.add(a, b)) === -1);
  assert(Point.y(Point.add(a, b)) === 9);
  assert(Point.x(Point.add(b, c)) === 0);
  assert(Point.y(Point.add(b, c)) === -2);
};

const testPointKeyIsAnInjection = () => {
  const n = 100;
  const map: Map<int, Point> = new Map();

  for (let x = -n; x <= n; x++) {
    for (let y = -n; y <= n; y++) {
      const point = Point.make(x, y);
      const match = map.get(Point.key(point));
      assert(!match, () => `Key collision: ${match}, ${Point.show(point)}`);
      map.set(Point.key(point), point);
    }
  }

  const side = 2 * n + 1;
  assert(map.size === side * side);
};

//////////////////////////////////////////////////////////////////////////////

const testHashTable = () => {
  const n = 100;
  const table = new HashTable();
  for (let i = 0; i < n; i++) {
    table.set(i, i * i);
  }
  for (let i = 0; i < n; i++) {
    assert(table.get(i) === i * i);
  }

  const keys: int[] = [];
  for (let x = -n; x <= n; x++) {
    for (let y = -n; y <= n; y++) {
      keys.push(Point.key(Point.make(x, y)));
    }
  }

  bench('Native Map set',  () => {
    const table = new Map();
    for (const key of keys) table.set(key, key / 2);
  });
  bench('Hash table set',  () => {
    const table = new HashTable(2 * keys.length);
    for (const key of keys) table.set(key, key / 2);
  });

  const t0 = new Map();
  const t1 = new HashTable();
  keys.forEach((key, i) => {
    t0.set(key, i);
    t1.set(key, i);
  });

  bench('Native Map get',  () => {
    for (let i = 0; i < keys.length; i++) assert(t0.get(keys[i]!) === i);
  });
  bench('Hash table get',  () => {
    for (let i = 0; i < keys.length; i++) assert(t1.get(keys[i]!) === i);
  });
};

//////////////////////////////////////////////////////////////////////////////

type SearchTestCase = {
  source: Point,
  target: Point,
  check: (p: Point) => Status,
  label: string,
  expected: string,
  raw: string[][],
};

const parseSearchTestCase = (input: string): SearchTestCase => {
  const split = input.trim().split('\n');
  const label = nonnull(split[0]).replace(':', '').replace(/-/g, ' ');
  const lines = split.slice(1);
  assert(lines.length > 0);
  const size = Point.make(nonnull(lines[0]).length, lines.length);
  const map = new Matrix(size, Status.FREE);
  const raw = range(Point.y(size)).map(_ => range(Point.x(size)).map(_ => '.'));
  const sources: Point[] = [];
  const targets: Point[] = [];

  lines.forEach((line, y) => {
    assert(line.length === Point.x(size));
    for (let x = 0; x < Point.x(size); x++) {
      const ch = nonnull(line[x]);
      const point = Point.make(x, y);
      if (ch === '.' || ch === '?' || ch === '*') continue;
      switch (ch) {
        case '@': sources.push(point); break;
        case 'T': targets.push(point); break;
        case 'X': map.set(point, Status.OCCUPIED); break;
        case '#': map.set(point, Status.BLOCKED); break;
        default: assert(false, () => `Unknown character: ${ch}`);
      }
      raw[y]![x] = ch;
    }
  });

  const check = (p: Point) => {
    const result = map.getOrNull(p);
    return result === null ? Status.BLOCKED : result;
  };
  const source = only(sources);
  const target = only(targets);
  const expected = lines.join('\n');
  return {source, target, check, label, expected, raw};
};

const runAStarTestCase = (input: string) => {
  const test = parseSearchTestCase(input);
  const {source, target, check, label, expected, raw} = test;

  const set = (point: Point, ch: string) => {
    const x = Point.x(point);
    const y = Point.y(point);
    const old = nonnull(nonnull(raw[y])[x]);
    if (old === '@' || old === 'T' || old === 'X' || old === '#') return;
    raw[y]![x] = ch;
  };

  const seen: Point[] = [];
  const path = nonnull(AStar(source, target, check, seen));
  bench(label, () => AStar(source, target, check));
  seen.forEach(x => set(x, '?'));
  path.forEach(x => set(x, '*'));

  const actual = raw.map(x => x.join('')).join('\n');
  assert(actual === expected,
         () => `${label}:\n\nActual:\n\n${actual}\n\nExpected:\n\n${expected}`);
};

const testAStar = () => {
  const cases = `
-Line of sight:
........
.....*T.
...**...
.@*.....
........

Occupied cells:
........
....X...
.@*XX*T.
...**...
........

-----Channel A:
...****...
..*####*..
.@??X...T.
..?####...
..........

-----Channel B:
..?****...
.?*####*..
??*####*..
?@??X...T.
???####...
.??####...
...???....

-----Channel C:
..????....
.??####...
???####...
???####...
?@**X***T.
???####...
???####...
.??####...
..????....

-----One block:
..............
......**......
....?*##**T...
...@*?##......
....??##......
..............

Several blocks:
.......................
..............##.......
..............##..**T..
.....##...********.....
.....##***##.....##....
..@****...###....##....
...........##..........
.......................

--Large meadow:
...............#..............#.....
.T**..................##.......##...
...#*....#............##.......##...
.....***...#......#.................
.......#*.......................#...
.........**#..##....................
...........***##...............##...
..............*##....###....#.......
...............***...######.#.......
...##...........##**********..#....#
...##.#.......######..######*.......
................????######???*......
................??????????????*.....
...#.............###???????????*....
.................##.??????????#?*...
.........................???????#*#.
...........................???????@.
....................................

--Large forest:
.##.##..#...###.###.#...#####.....#.##.
...###..#*?..####...#..##..#.##.#.....#
######***#***?##..#.....###.#...##.....
#..T**#??##?#***?????..####.#...###...#
#.#....#???.#?##*?##??####.####.####..#
.#....#.##..###?#*??#???##?##..##...#..
#...##.#.#.##.##??***?##??#?#####..##..
#....#..#.#.#..###?##***?##????#.##...#
#...#.#..#....#.##.#?#?#****##??.......
##..#.##...#...#...#.#.#####*##??..##.#
####...##...#.####.###.######*****...#.
..#..##.......##..##...##...#????#*####
.#..####..##..#?#####.########?####*.##
........#..#..#??????.#######????#?@.##
.#.###.......#.#...##?#??###??###?#?#.#
##....#.#....##.#..##???#??#?#..##?.#.#
#..###..####.#.....##..#?#??#.##..####.
  `;
  cases.split('\n\n').forEach(runAStarTestCase);
};

//////////////////////////////////////////////////////////////////////////////

const main = () => {
  testBasicPointOperations();
  testPointKeyIsAnInjection();
  testHashTable();
  testAStar();
};

main();

export {}
