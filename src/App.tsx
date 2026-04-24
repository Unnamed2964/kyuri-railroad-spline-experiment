import { TwoPointStraightPadSolverPage } from './components/TwoPointStraightPadSolverPage';

export default function App() {
  return (
    <main className="page-shell app-shell">
      <section className="page-header app-header">
        <p className="eyebrow">railroad spline experiment</p>
        <h1>铁路缓和曲线实验</h1>
        <p className="lead">
          给定两点、两端切线和参数 A，实时搜索允许首尾补直线时的最大可行半径 R。
        </p>
      </section>

      <TwoPointStraightPadSolverPage />
    </main>
  );
}