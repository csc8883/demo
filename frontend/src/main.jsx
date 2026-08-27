import React from 'react';
import { createRoot } from 'react-dom/client';
import App from './App.jsx';
import 'sweetalert2/dist/sweetalert2.min.css';
import './styles/tailwind.css';
import './styles/tailwind-fallback.css';
import './styles/style.css';
import './styles/theme-extra.css';
import './styles/ui-refresh.css';
import './styles/weight-profile.css';
import './styles/react-shell.css';
import './styles/theme-dark.css';

createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
