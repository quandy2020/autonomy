/** Mosaic leaf id may be `image` or `image#3` for multi-instance panels. */
export function panelBaseId(id: string): string {
  const i = id.indexOf('#');
  return i >= 0 ? id.slice(0, i) : id;
}

/** Next free instance id for a multi-open panel type (e.g. image → image#2). */
export function allocPanelInstanceId(base: string, existing: string[]): string {
  const taken = new Set(existing);
  if (!taken.has(base)) return base;
  let n = 2;
  while (taken.has(`${base}#${n}`)) n += 1;
  return `${base}#${n}`;
}
