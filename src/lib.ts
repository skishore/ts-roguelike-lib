type int = number;

//////////////////////////////////////////////////////////////////////////////
// A few utility functions.

const assert = (x: boolean, fn?: () => string): void => {
  if (!x) throw new Error(fn && fn());
};

const nonnull = <T>(x: T): NonNullable<T> => {
  if (x === null || x === undefined) throw new Error();
  return x as NonNullable<T>;
};

const only = <T>(xs: T[]): T => {
  assert(xs.length === 1);
  return xs[0]!;
};

const range = (n: int): int[] => {
  n = n | 0;
  const result = [];
  for (let i = 0; i < n; i++) {
    result.push(i);
  }
  return result;
};

//////////////////////////////////////////////////////////////////////////////
// Benchmarking and testing - move it to another file soon.

declare const console: any;

const bench = (name: string, fn: () => void): void => {
  for (let i = 0; i < 1e3; i++) fn();

  let delta = 0;
  let count = 1e3;
  for (; delta < 1e3; count *= 2) {
    const start = Date.now();
    for (let i = 0; i < count; i++) fn();
    delta = Date.now() - start;
  }

  const speed = Math.floor(1e6 * delta / count).toLocaleString();
  const extra = ' '.repeat(11 - speed.length);
  console.log(`${name}: ${extra}${speed} ns/iter (n = ${count})`);
};

//////////////////////////////////////////////////////////////////////////////

export {assert, bench, int, nonnull, only, range};
