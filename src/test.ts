import {assert, int, nonnull, range} from './lib';
import {Point, Matrix, AStar, Status} from './geo';

//////////////////////////////////////////////////////////////////////////////

const testPointKeyIsAnInjection = () => {
  const n = 100;
  const map: Map<int, Point> = new Map();

  for (let x = -n; x <= n; x++) {
    for (let y = -n; y <= n; y++) {
      const point = new Point(x, y);
      const match = map.get(point.key());
      assert(!match, () => `Point.key collision: ${match}, ${point}`);
      map.set(point.key(), point);
    }
  }

  const side = 2 * n + 1;
  assert(map.size === side * side);
};

//////////////////////////////////////////////////////////////////////////////

type SearchTestCase = {
  source: Point,
  targets: Point[],
  check: (p: Point) => Status,
  raw: string[][],
};

const parseSearchTestCase = (input: string): SearchTestCase => {
  const lines = input.trim().split('\n');
  assert(lines.length > 0);
  const size = new Point(nonnull(lines[0]).length, lines.length);
  const map = new Matrix(size, Status.FREE);
  const raw = range(size.y).map(_ => range(size.x).map(_ => '.'));
  const sources: Point[] = [];
  const targets: Point[] = [];

  lines.forEach((line, y) => {
    assert(line.length === size.x);
    for (let x = 0; x < size.x; x++) {
      const ch = nonnull(line[x]);
      const point = new Point(x, y);
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

  assert(sources.length === 1);
  return {source: nonnull(sources[0]), targets, check, raw};
};

const runAStarTestCase = (input: string) => {
  const expected = input.trim();
  const test = parseSearchTestCase(input);
  assert(test.targets.length === 1);
  const target = nonnull(test.targets[0]);

  const set = (point: Point, ch: string) => {
    const {x, y} = point;
    const old = nonnull(nonnull(test.raw[y])[x]);
    if (old === '@' || old === 'T' || old === 'X' || old === '#') return;
    test.raw[y]![x] = ch;
  };

  const seen: Point[] = [];
  const path = nonnull(AStar(test.source, target, test.check, seen));
  seen.forEach(x => set(x, '?'));
  path.forEach(x => set(x, '*'));

  const actual = test.raw.map(x => x.join('')).join('\n');
  assert(actual === expected,
         () => `Actual:\n\n${actual}\n\nExpected:\n\n${expected}`);
};

const testAStar = () => {
  const cases = `
........
.....*T.
...**...
.@*.....
........

........
..??X...
.@*XX*T.
..?**...
........

...???....
..?####...
.@??X...T.
..*####*..
...****...

...****...
.?*####*..
??*####.*.
?@??X...T.
???####...
.??####...
...???....

..????....
.??####...
???####...
???####...
?@**X***T.
???####...
???####...
.??####...
..????....

..............
......**......
....?*##**T...
...@*?##......
....??##......
..............

.......................
..............##.......
..............##..**T..
.....##.??********.....
...??##***##.....##....
..@****???###....##....
...........##..........
.......................

...............#..............#.....
.T**?.................##.......##...
...#*??..#............##.......##...
....?***?..#......#.................
......?#*?......................#...
.......??**#..##....................
...........***##...............##...
..............*##?...###....#.......
..............?***??.######.#.......
...##...........##**********..#....#
...##.#.......######..######*.......
...............?????######???*......
.................?????????????*.....
...#.............###???????????*....
.................##..?????????#?*...
.........................???????#*#.
..........................????????@.
....................................

.##.##..#...###.###.#...#####.....#.##.
...###..#*?..####...#..##..#.##.#.....#
######***#***?##..#.....###.#...##.....
#..T**#??##?#***?????..####.#...###...#
#.#...?#???.#?##*?##??####.####.####..#
.#....#.##..###?#*??#???##?##..##...#..
#...##.#.#.##.##??***?##??#?#####..##..
#....#..#.#.#..###?##***?##????#.##...#
#...#.#..#....#.##.#?#?#****##??.......
##..#.##...#...#...#.#.#####*##??..##.#
####...##...#.####.###.######*****...#.
..#..##.......##..##...##...#????#*####
.#..####..##..#.#####.########?####*.##
........#..#..#..????.#######????#?@.##
.#.###.......#.#...##?#??###??###?#?#.#
##....#.#....##.#..##??.#??#?#..##?.#.#
#..###..####.#.....##..#.#??#.##..####.
  `;
  cases.split('\n\n').forEach(runAStarTestCase);
};

//////////////////////////////////////////////////////////////////////////////

const main = () => {
  testPointKeyIsAnInjection();
  testAStar();
};

main();

export {}
