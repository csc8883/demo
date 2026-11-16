import { useSystemPortal } from './SystemPortal.jsx';
import './assistant-pet.css';

export default function AssistantPet() {
  const portal = useSystemPortal(import.meta.env.VITE_ASSISTANT_URL, 'http://127.0.0.1:5173/', '巡检知识助手');
  return <><div className='assistant-pet-dock'>
    <div className='assistant-pet-bubble' aria-hidden='true'>需要帮忙？一起去知识空间 <span>↗</span></div>
    <button type='button' className='assistant-pet' aria-label='进入巡检知识助手' title='进入巡检知识助手' aria-busy={portal.busy} disabled={portal.busy} onClick={portal.open}>
      <span className='assistant-pet-halo' />
      <svg viewBox='0 0 120 128' fill='none' aria-hidden='true'>
        <defs><linearGradient id='pet-shell' x1='22' y1='24' x2='99' y2='115' gradientUnits='userSpaceOnUse'><stop stopColor='#f0fdff'/><stop offset='1' stopColor='#9bd9df'/></linearGradient></defs>
        <ellipse className='pet-shadow' cx='60' cy='119' rx='30' ry='5' fill='#0f766e' opacity='.2'/>
        <g className='pet-float'><path d='M60 27V16' stroke='#99f6e4' strokeWidth='4'/><circle cx='60' cy='12' r='5' fill='#5eead4'/>
          <rect x='13' y='48' width='13' height='25' rx='6' fill='#5eead4'/><rect x='94' y='48' width='13' height='25' rx='6' fill='#5eead4'/>
          <rect x='23' y='28' width='74' height='63' rx='25' fill='url(#pet-shell)' stroke='#fff' strokeWidth='2'/>
          <rect x='31' y='39' width='58' height='39' rx='16' fill='#102d3b'/>
          <g className='pet-eyes'><rect x='42' y='49' width='9' height='15' rx='4.5' fill='#7df9e5'/><rect x='69' y='49' width='9' height='15' rx='4.5' fill='#7df9e5'/></g>
          <path d='M55 67Q60 71 65 67' stroke='#7df9e5' strokeWidth='2' strokeLinecap='round'/>
          <path d='M40 91Q60 84 80 91L76 105Q60 116 44 105Z' fill='url(#pet-shell)'/><circle cx='60' cy='99' r='5' fill='#14b8a6'/>
          <path className='pet-hand' d='M86 88Q102 81 101 74' stroke='#b8ecec' strokeWidth='9' strokeLinecap='round'/>
        </g>
      </svg>
      <span className='assistant-pet-label'><i />{portal.checking ? '正在连接…' : '知识助手'}<span>↗</span></span>
    </button>
  </div>{portal.feedback}</>;
}
