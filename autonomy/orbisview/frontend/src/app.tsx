/**
 * OrbisView entry.
 */
import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { Orbisview } from '@/components/Orbisview';
import { registerBuiltinPanels } from '@/components';
import '@/styles/main.css';

registerBuiltinPanels();

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <Orbisview />
  </StrictMode>,
);
