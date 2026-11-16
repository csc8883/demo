import { useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import './system-portal.css';
const marker = '#system-portal';
const reducedMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
export function PortalArrival() {
    const [arriving, setArriving] = useState(() => window.location.hash === marker);
    useEffect(() => {
        if (!arriving)
            return;
        window.history.replaceState(window.history.state, '', window.location.pathname + window.location.search);
        const timer = window.setTimeout(() => setArriving(false), reducedMotion() ? 80 : 850);
        return () => window.clearTimeout(timer);
    }, [arriving]);
    return arriving ? createPortal(<div className='system-portal-arrival' aria-hidden='true'/>, document.body) : null;
}
export function useSystemPortal(configuredUrl, fallback, label) {
    const [phase, setPhase] = useState('idle');
    const [error, setError] = useState('');
    const locked = useRef(false);
    const controller = useRef(null);
    const timer = useRef();
    const trigger = useRef(null);
    useEffect(() => {
        const reset = () => {
            locked.current = false;
            setPhase('idle');
            document.documentElement.classList.remove('system-portal-leaving');
        };
        window.addEventListener('pageshow', reset);
        return () => {
            window.removeEventListener('pageshow', reset);
            controller.current?.abort();
            window.clearTimeout(timer.current);
            document.documentElement.classList.remove('system-portal-leaving');
        };
    }, []);
    async function open(event) {
        if (locked.current)
            return;
        locked.current = true;
        trigger.current = event.currentTarget;
        setError('');
        setPhase('checking');
        const rect = event.currentTarget.getBoundingClientRect();
        document.documentElement.style.setProperty('--portal-x', `${rect.left + rect.width / 2}px`);
        document.documentElement.style.setProperty('--portal-y', `${rect.top + rect.height / 2}px`);
        const abort = new AbortController();
        controller.current = abort;
        const timeout = window.setTimeout(() => abort.abort(), 3500);
        try {
            const target = new URL(configuredUrl || fallback);
            if (!['http:', 'https:'].includes(target.protocol))
                throw new Error('invalid target');
            if (target.origin === window.location.origin)
                throw new Error('same origin');
            // A simple navigation probe works across the two local origins without requiring CORS headers.
            const response = await fetch(target.href, { mode: 'no-cors', credentials: 'omit', cache: 'no-store', signal: abort.signal });
            if (response.type !== 'opaque' && !response.ok)
                throw new Error('unavailable');
            window.clearTimeout(timeout);
            if (abort.signal.aborted)
                return;
            setPhase('leaving');
            document.documentElement.classList.add('system-portal-leaving');
            target.hash = marker;
            timer.current = window.setTimeout(() => window.location.assign(target.href), reducedMotion() ? 80 : 780);
        }
        catch {
            if (controller.current !== abort)
                return;
            setError(`暂时无法进入${label}。请确认目标系统已启动，地址为 ${configuredUrl || fallback}。`);
            setPhase('idle');
            locked.current = false;
        }
        finally {
            window.clearTimeout(timeout);
        }
    }
    const feedback = createPortal(<>
    {phase === 'leaving' && <div className='system-portal-overlay' role='status' aria-live='polite'><div className='system-portal-orbit'/><div className='system-portal-caption'><span>系统联动</span><strong>正在进入{label}</strong><small>连接你的巡检工作空间</small></div></div>}
    {error && <div className='system-portal-error' role='alert'><strong>目标系统尚未就绪</strong><p>{error}</p><button type='button' onClick={() => { setError(''); trigger.current?.focus(); }}>知道了</button></div>}
  </>, document.body);
    return { open, busy: phase !== 'idle', checking: phase === 'checking', feedback };
}
