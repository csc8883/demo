window.filterAdminTable = (query = '') => {
  const normalized = String(query).toLowerCase();
  document.querySelectorAll('#admin-content tbody tr').forEach((row) => {
    row.style.display = row.innerText.toLowerCase().includes(normalized) ? '' : 'none';
  });
};

let bootPromise = null;

export const actions = new Proxy(
  {},
  {
    get(_target, property) {
      return window[property];
    },
  },
);

export function bootApp() {
  if (!bootPromise) {
    bootPromise = Promise.all([
      import('chart.js/auto'),
      import('sweetalert2'),
      import('@floating-ui/dom'),
      import('@phosphor-icons/web/bold'),
      import('@phosphor-icons/web/fill'),
      import('@phosphor-icons/web/duotone'),
    ]).then(([chartModule, swalModule, floatingModule]) => {
      window.Chart = chartModule.default;
      window.Swal = swalModule.default;
      window.FloatingUIDOM = floatingModule;
      return import('./app.js');
    });
  }
  return bootPromise;
}
