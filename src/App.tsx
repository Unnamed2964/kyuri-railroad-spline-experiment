import { TwoPointStraightPadSolverPage } from './components/TwoPointStraightPadSolverPage';

export default function App() {
  return (
    <main className="page-shell app-shell">
      <section className="page-header app-header">
        <p className="eyebrow">railroad spline experiment</p>
        <h1>铁路缓和曲线实验</h1>
        <p className="app-disclaimer" role="note" aria-label="免责声明">
          <span className="app-disclaimer-label">注意</span>
          <span className="app-disclaimer-body">
            这是一个自娱自乐的 AI vibe coding 小实验。代码、数学推导、结果和文案都没有经过认真审查，请不要把它当成可靠工具。
          </span>
        </p>
        <p className="lead">
          给定两点、两端切线和参数 A，实时搜索允许首尾补直线时的最大可行半径 R。
        </p>
        <div className="inline-links" aria-label="外部链接">
          <a href="https://railroad-spline-experiment.umamichi.moe/" target="_blank" rel="noreferrer">
            在线体验
          </a>
          <a href="https://github.com/Unnamed2964/kyuri-railroad-spline-experiment" target="_blank" rel="noreferrer">
            GitHub 仓库
          </a>
          <a href="https://umamichi.moe/" target="_blank" rel="noreferrer">
            个人主页
          </a>
        </div>
      </section>

      <TwoPointStraightPadSolverPage />
    </main>
  );
}