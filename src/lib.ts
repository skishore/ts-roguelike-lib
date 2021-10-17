type int = number;

//////////////////////////////////////////////////////////////////////////////
// A few utility functions.

const assert = (x: boolean, fn?: () => string): void => {
  if (!x) throw new Error(fn && fn());
};

const nonnull = <T>(x: T): NonNullable<T> => {
  if (x === null || x === undefined) throw new Error();
  return x as NonNullable<T>;
}

const range = (n: int): int[] => {
  n = n | 0;
  const result = [];
  for (let i = 0; i < n; i++) {
    result.push(i);
  }
  return result;
};

//////////////////////////////////////////////////////////////////////////////

export {assert, int, nonnull, range};
